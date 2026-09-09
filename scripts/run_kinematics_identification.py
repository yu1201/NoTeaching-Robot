"""Read-only sampling -> offline fit -> independent validation, no deployment.

Experimental evidence and candidate models live in a NEW SQLite database. This
tool never opens production ConfigStore.db. Reports are derived exports only.
Only the InovanceReference backend knows its protocol; the workflow is reusable.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np

from inovance_kinematics_reference import InovanceReference
from kinematics_model_fit import (corrections_to_links, error_metrics, fit_chain, forward,
                                 inverse_local, pose_matrix, fit_flexible_chain,
                                 evaluate_model, flexibility_offsets)


def stamp():
    return datetime.now(timezone.utc).isoformat()


def encoded(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def put_meta(db, key, value):
    db.execute("INSERT OR REPLACE INTO metadata VALUES (?,?)", (key, encoded(value)))
    db.commit()


def get_meta(db, key):
    row = db.execute("SELECT value FROM metadata WHERE key=?", (key,)).fetchone()
    if row is None:
        raise ValueError("Missing experiment metadata: " + key)
    return json.loads(row[0])


def connect_db(path, writable=False):
    path = Path(path).resolve()
    if path.name != 'experiment.sqlite':
        raise ValueError("Only an experiment.sqlite database is accepted; never ConfigStore.db")
    db = sqlite3.connect(path.as_uri() + ('?mode=rw' if writable else '?mode=ro'), uri=True)
    if get_meta(db, 'schema') != 'robot-kinematics-experiment-v1':
        db.close(); raise ValueError("Unrecognized experiment database")
    return db


def make_design(limits, current, seed=20260906, training_count=240):
    """Freeze train/test regions before querying any reference values.

    Regional holdout excludes an entire high-J2/high-J5 band from training;
    trajectories and repeatability samples are also never fitted.
    """
    if not 80 <= training_count <= 4000:
        raise ValueError("Training count must be 80..4000")
    rng = np.random.default_rng(seed)
    limits = np.asarray(limits); margin = np.minimum(8., .05*(limits[:, 1]-limits[:, 0]))
    lo, hi = limits[:, 0]+margin, limits[:, 1]-margin
    design = []; seen = set()
    def add(label, q, allow_duplicate=False):
        q = np.round(q, 3)
        if np.any(q <= limits[:, 0]) or np.any(q >= limits[:, 1]):
            raise ValueError("Sample outside limit interior")
        key = tuple(q)
        if key in seen and not allow_duplicate:
            return False
        seen.add(key); design.append((label, q.tolist())); return True
    for label, count in [('train', training_count), ('test_random', max(40, training_count//3)),
                         ('test_region', max(40, training_count//3))]:
        added = 0
        while added < count:
            u = rng.uniform(0, 1, 6)
            if label == 'test_region':
                axis = 1 if added % 2 == 0 else 4
                u[axis] = rng.uniform(.72, 1.)
            else:
                u[[1, 4]] *= .65
            added += add(label, lo+u*(hi-lo))
    for t in np.linspace(-1, 1, 41):
        q = np.clip(np.asarray(current) + t*np.array([2., -3., 3., 5., 4., -5.]), lo, hi)
        add('test_path', q)
    for _ in range(5):
        add('repeat_current', current, allow_duplicate=True)
    return design


def identity(snapshot):
    # Current pose is not model identity. Active context is monitored separately.
    return {k: snapshot[k] for k in ('host', 'port', 'model', 'firmware', 'machine_sha256',
                                     'contexts', 'limits_deg', 'active_tool', 'active_wobj')}


def reference(args):
    password = os.environ.get('INOVANCE_FTP_PASSWORD')
    if password is None:
        raise ValueError("Set INOVANCE_FTP_PASSWORD in the process environment; it is never logged")
    return InovanceReference(args.host, args.port, args.ftp_port, args.ftp_user, password, args.rate)


def collect(args):
    directory = Path(args.output).resolve()
    directory.mkdir(parents=True, exist_ok=False)  # no overwrite / implicit resume
    db = sqlite3.connect(directory / 'experiment.sqlite')
    db.executescript('''
        CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL);
        CREATE TABLE protocol_log(id INTEGER PRIMARY KEY,time TEXT,request TEXT,reply TEXT);
        CREATE TABLE samples(id INTEGER PRIMARY KEY,profile TEXT,split TEXT,joints TEXT,payload TEXT);
        CREATE TABLE native_ik(id INTEGER PRIMARY KEY,sample_id INTEGER,payload TEXT);
        CREATE TABLE candidates(profile TEXT PRIMARY KEY,payload TEXT);
    ''')
    put_meta(db, 'schema', 'robot-kinematics-experiment-v1')
    put_meta(db, 'state', 'collecting'); put_meta(db, 'started', stamp())
    put_meta(db, 'robot_label', args.robot_label)
    put_meta(db, 'script_sha256', {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                 for name in ['run_kinematics_identification.py',
                                              'kinematics_model_fit.py', 'inovance_kinematics_reference.py']})
    profiles = [tuple(map(int, s.split(','))) for s in args.profiles]
    if not profiles or any(len(p) != 3 for p in profiles) or len(set(profiles)) != len(profiles):
        raise ValueError("Expected distinct tool,wobj,load profiles")
    try:
        with reference(args) as reader:
            def audit(command, reply):
                db.execute('INSERT INTO protocol_log(time,request,reply) VALUES (?,?,?)', (stamp(), command, reply))
                db.commit()
            reader.audit = audit
            snapshot = reader.snapshot(profiles)
            put_meta(db, 'snapshot_start', snapshot)
            put_meta(db, 'source_binding_sha256', hashlib.sha256(encoded(identity(snapshot)).encode()).hexdigest())
            design = make_design(snapshot['limits_deg'], snapshot['current_joints'], args.seed, args.training_count)
            put_meta(db, 'design', design); put_meta(db, 'seed', args.seed)
            put_meta(db, 'expected_samples', len(design)*len(profiles))
            put_meta(db, 'gate', {'position_max_mm': .05, 'orientation_max_deg': .01,
                                'meaning': 'experimental numerical agreement only, not physical accuracy'})
            total = len(design)*len(profiles); done = 0; start = time.monotonic(); inverse_cases = {}
            for i, (split, q) in enumerate(design):
                if i % 25 == 0:
                    reader.assert_stopped()
                for p in profiles:
                    key = ','.join(map(str, p)); result = reader.forward(q, key)
                    cursor = db.execute('INSERT INTO samples(profile,split,joints,payload) VALUES (?,?,?,?)',
                                        (key, split, encoded(result['joint_deg']), encoded(result)))
                    db.commit(); done += 1
                    if split == 'test_random' and len(inverse_cases.get(key, [])) < 12:
                        inverse_cases.setdefault(key, []).append((cursor.lastrowid, result))
                if i % 20 == 0 or done == total:
                    print(f"SAMPLE {done}/{total}, elapsed={time.monotonic()-start:.1f}s", flush=True)
            for key, cases in inverse_cases.items():
                for sample_id, result in cases:
                    inv = reader.inverse(result, key)
                    closure = reader.forward(inv['joint_deg'], key)
                    db.execute('INSERT INTO native_ik(sample_id,payload) VALUES (?,?)',
                               (sample_id, encoded({'inverse': inv, 'closure': closure})))
                    db.commit()
            end = reader.snapshot(profiles)
            put_meta(db, 'snapshot_end', end)
            if identity(snapshot) != identity(end):
                raise ValueError("Controller/context changed during sampling; dataset is not eligible for fit")
            if db.execute('SELECT COUNT(*) FROM samples').fetchone()[0] != total:
                raise ValueError("Incomplete sampling")
            put_meta(db, 'state', 'complete'); put_meta(db, 'completed', stamp())
            print(f"COMPLETE: {directory / 'experiment.sqlite'}", flush=True)
    except BaseException as error:
        put_meta(db, 'state', 'incomplete'); put_meta(db, 'error', type(error).__name__ + ': ' + str(error))
        raise
    finally:
        db.close()


def fit(args):
    db = connect_db(args.database, writable=True)
    try:
        if get_meta(db, 'state') != 'complete':
            raise ValueError("Incomplete / source-changed experiment cannot be fitted")
        if db.execute('SELECT COUNT(*) FROM candidates').fetchone()[0]:
            raise ValueError("Candidates already exist; never silently overwrite a prior fit")
        snapshot = get_meta(db, 'snapshot_start'); nominal = np.array(snapshot['nominal_links'])
        if args.method == 'gravity' and any(abs(snapshot['install'][key]) > 1e-8 for key in ['gravAlpha','gravBeta','gravGamma']):
            raise ValueError('Nonzero installation gravity angles are outside this pilot model scope')
        limits = np.array(snapshot['limits_deg']); gate = get_meta(db, 'gate')
        report = {'schema': 'kinematics-identification-report-v1', 'created': stamp(),
                  'robot_label': get_meta(db, 'robot_label'), 'model': snapshot['model'],
                  'firmware': snapshot['firmware'], 'binding': get_meta(db, 'source_binding_sha256'),
                  'method': args.method,
                  'fit_code_sha256': hashlib.sha256(Path(__file__).with_name('kinematics_model_fit.py').read_bytes()).hexdigest(),
                  'production_enabled': False, 'physical_accuracy_verified': False, 'profiles': {}}
        for profile, context in snapshot['contexts'].items():
            rows = db.execute('SELECT id,split,joints,payload FROM samples WHERE profile=? ORDER BY id', (profile,)).fetchall()
            q = np.array([json.loads(row[2]) for row in rows]); splits = np.array([row[1] for row in rows])
            tcp = np.array([pose_matrix(json.loads(row[3])['pose']) for row in rows])
            tool = np.array(context['tool_matrix']); work = np.array(context['work_matrix'])
            target = work @ tcp @ np.linalg.inv(tool)
            train = splits == 'train'
            print(f"FIT {profile}: {train.sum()} training, {len(rows)-train.sum()} untouched evaluation", flush=True)
            flexibility = None
            if args.method == 'gravity':
                parameters, flexibility, details = fit_flexible_chain(q[train], target[train], nominal)
            else:
                parameters, details = fit_chain(q[train], target[train], nominal)
            baseline = forward(q, nominal); fitted = evaluate_model(q, nominal, parameters, flexibility)
            if flexibility is not None:
                details['max_all_samples_deflection_mrad'] = float(np.max(np.abs(flexibility_offsets(q, nominal, flexibility))))
            metrics = {}
            for split in sorted(set(splits)):
                pick = splits == split
                metrics[split] = {'baseline_flange': error_metrics(baseline[pick], target[pick]),
                                  'fitted_flange': error_metrics(fitted[pick], target[pick]),
                                  'baseline_tcp': error_metrics(np.linalg.inv(work) @ baseline[pick] @ tool, tcp[pick]),
                                  'fitted_tcp': error_metrics(np.linalg.inv(work) @ fitted[pick] @ tool, tcp[pick])}
            ik_checks = []
            for idx in np.flatnonzero(splits == 'test_random')[:12]:
                seed = np.clip(q[idx] + [.2, -.2, .15, -.15, .2, -.2], limits[:, 0], limits[:, 1])
                solved, ok = inverse_local(target[idx], seed, nominal, parameters, limits, flexibility=flexibility)
                ik_checks.append({'sample_id': rows[idx][0], 'converged': ok, 'joint_deg': solved.tolist(),
                                  'same_model_closure': error_metrics(evaluate_model(solved, nominal, parameters, flexibility), target[idx:idx+1])})
            native_checks = []
            for source, payload in db.execute('''SELECT s.payload,n.payload FROM native_ik n
                    JOIN samples s ON n.sample_id=s.id WHERE s.profile=?''', (profile,)):
                source, payload = json.loads(source), json.loads(payload)
                native_checks.append(error_metrics(pose_matrix(payload['closure']['pose'])[None],
                                                   pose_matrix(source['pose'])[None]))
            # Preliminary thresholds were frozen before sampling. Passing them does
            # not enable production or imply collision/branch/workspace safety.
            passed = not details['bound_hit'] and all(x['converged'] for x in ik_checks)
            if flexibility is not None:
                passed &= details['max_all_samples_deflection_mrad'] < 5.
            for split in ['test_random', 'test_region', 'test_path', 'repeat_current']:
                for frame in ['fitted_flange', 'fitted_tcp']:
                    m = metrics[split][frame]
                    passed &= m['position_mm']['max'] <= gate['position_max_mm'] and m['orientation_deg']['max'] <= gate['orientation_max_deg']
            base, links = corrections_to_links(nominal, parameters)
            candidate = {'schema': 'equivalent-6r-chain-v1', 'production_enabled': False,
                         'source_binding': report['binding'], 'context': context,
                         'method': args.method, 'flexibility': flexibility,
                         'corrections_mm_mrad': parameters.tolist(), 'base': base.tolist(), 'links': links.tolist(),
                         'fit_details': details, 'metrics': metrics, 'local_ik_checks': ik_checks,
                         'controller_native_roundtrip_metrics': native_checks,
                         'independent_controller_ik_validation': 'not_yet_run',
                         'preliminary_numerical_gate_passed': bool(passed), 'joint_limits_deg': limits.tolist()}
            db.execute('INSERT INTO candidates VALUES (?,?)', (profile, encoded(candidate))); db.commit()
            report['profiles'][profile] = candidate
            m = metrics['test_region']['fitted_tcp']
            print(f"RESULT {profile}: regional TCP max={m['position_mm']['max']:.6f}mm / {m['orientation_deg']['max']:.6f}deg, gate={passed}", flush=True)
        put_meta(db, 'fit_report', report)
        export_report(Path(args.database).resolve().parent, report)
    finally:
        db.close()


def validate_controller(args):
    """Send local IK solutions ONLY to controller FK, never to a motion interface."""
    db = connect_db(args.database, writable=True)
    try:
        if get_meta(db, 'state') != 'complete':
            raise ValueError('Sampling incomplete')
        report = get_meta(db, 'fit_report'); snapshot = get_meta(db, 'snapshot_start')
        if args.host != snapshot['host'] or args.port != snapshot['port']:
            raise ValueError('Endpoint differs from sampled controller')
        if any(c['independent_controller_ik_validation'] != 'not_yet_run' for c in report['profiles'].values()):
            raise ValueError('Independent verification already recorded; refusing overwrite')
        profiles = [tuple(map(int, key.split(','))) for key in report['profiles']]
        put_meta(db, 'controller_validation_state', 'running')
        with reference(args) as reader:
            def audit(command, reply):
                db.execute('INSERT INTO protocol_log(time,request,reply) VALUES (?,?,?)', (stamp(), command, reply))
                db.commit()
            reader.audit = audit
            if identity(reader.snapshot(profiles)) != identity(snapshot):
                raise ValueError('Source/context changed since sampling')
            for profile, candidate in report['profiles'].items():
                checks = []
                for case in candidate['local_ik_checks']:
                    if not case['converged']:
                        checks.append({'sample_id': case['sample_id'], 'status': 'local_ik_not_converged'})
                        continue
                    source = json.loads(db.execute('SELECT payload FROM samples WHERE id=?', (case['sample_id'],)).fetchone()[0])
                    reader.assert_stopped()
                    response = reader.forward(case['joint_deg'], profile)
                    metrics = error_metrics(pose_matrix(response['pose'])[None], pose_matrix(source['pose'])[None])
                    checks.append({'sample_id': case['sample_id'], 'status': 'calculated_without_motion',
                                   'local_solution_deg': case['joint_deg'], 'controller_fk': response,
                                   'target': source, 'independent_tcp_metrics': metrics})
                candidate['independent_controller_ik_validation'] = {'checks': checks,
                    'scope': '12 seeded local branches; not all IK branches or collision safety',
                    'max_position_mm': max((c['independent_tcp_metrics']['position_mm']['max'] for c in checks
                                            if 'independent_tcp_metrics' in c), default=None),
                    'max_orientation_deg': max((c['independent_tcp_metrics']['orientation_deg']['max'] for c in checks
                                                if 'independent_tcp_metrics' in c), default=None)}
                print(f"CONTROLLER IK CHECK {profile}: {candidate['independent_controller_ik_validation']['max_position_mm']} mm", flush=True)
            if identity(reader.snapshot(profiles)) != identity(snapshot):
                raise ValueError('Source/context changed during verification')
        for profile, candidate in report['profiles'].items():
            v = candidate['independent_controller_ik_validation']
            gate = get_meta(db, 'gate')
            candidate['independent_controller_gate_passed'] = bool(
                all(c['status'] == 'calculated_without_motion' for c in v['checks'])
                and v['max_position_mm'] is not None and v['max_position_mm'] <= gate['position_max_mm']
                and v['max_orientation_deg'] <= gate['orientation_max_deg'])
            db.execute('UPDATE candidates SET payload=? WHERE profile=?', (encoded(candidate), profile))
        put_meta(db, 'fit_report', report); put_meta(db, 'controller_validation_state', 'complete')
        export_report(Path(args.database).resolve().parent, report)
    except BaseException:
        put_meta(db, 'controller_validation_state', 'incomplete')
        raise
    finally:
        db.close()


def validate_offsets(args):
    """Unseen Cartesian targets: avoid a trivial FK->IK->rounded-original-JP check."""
    db = connect_db(args.database, writable=True)
    try:
        snapshot=get_meta(db,'snapshot_start');report=get_meta(db,'fit_report')
        if get_meta(db,'state')!='complete' or args.host!=snapshot['host'] or args.port!=snapshot['port']:
            raise ValueError('Incomplete dataset or different endpoint')
        if any('offset_target_validation' in c for c in report['profiles'].values()):
            raise ValueError('Offset validation already exists; refusing overwrite')
        profiles=[tuple(map(int,k.split(','))) for k in report['profiles']]
        # Frozen before queries; targets are not used for any fitting.
        offset=np.array([.35,-.25,.2,.015,-.012,.018])
        put_meta(db,'offset_validation_spec',{'xyzabc_offset':offset.tolist(),'count_per_profile':12,
                 'target_rounding_decimals':3,'not_training_data':True})
        put_meta(db,'offset_validation_state','running')
        nominal=np.array(snapshot['nominal_links']);limits=np.array(snapshot['limits_deg'])
        with reference(args) as reader:
            def audit(command,reply):
                db.execute('INSERT INTO protocol_log(time,request,reply) VALUES (?,?,?)',(stamp(),command,reply));db.commit()
            reader.audit=audit
            if identity(reader.snapshot(profiles))!=identity(snapshot):
                raise ValueError('Controller/context changed since fitting')
            for profile,candidate in report['profiles'].items():
                context=candidate['context'];tool=np.array(context['tool_matrix']);work=np.array(context['work_matrix'])
                rows=db.execute("SELECT id,payload FROM samples WHERE profile=? AND split='test_random' ORDER BY id LIMIT 12",(profile,)).fetchall()
                checks=[]
                for sample_id,payload in rows:
                    source=json.loads(payload)
                    target=np.round(np.array(source['pose'])+offset,3);target_tcp=pose_matrix(target)
                    target_flange=work@target_tcp@np.linalg.inv(tool)
                    solved,ok=inverse_local(target_flange,source['joint_deg'],nominal,
                                            candidate['corrections_mm_mrad'],limits,flexibility=candidate['flexibility'])
                    if not ok:
                        checks.append({'sample_id':sample_id,'status':'local_inverse_failed'});continue
                    reader.assert_stopped()
                    local_fk=reader.forward(solved,profile)
                    native_ik=reader.inverse({'pose':target.tolist(),'arm':source['arm']},profile)
                    native_fk=reader.forward(native_ik['joint_deg'],profile)
                    checks.append({'sample_id':sample_id,'status':'calculated_without_motion',
                                   'target_xyzabc':target.tolist(),'local_joint_solution':solved.tolist(),
                                   'controller_fk_of_local_solution':local_fk,'controller_native_inverse':native_ik,
                                   'controller_fk_of_native_inverse':native_fk,
                                   'local_independent_metrics':error_metrics(pose_matrix(local_fk['pose'])[None],target_tcp[None]),
                                   'native_independent_metrics':error_metrics(pose_matrix(native_fk['pose'])[None],target_tcp[None])})
                good=[c for c in checks if c['status']=='calculated_without_motion']
                summary={'checks':checks,'case_count':len(checks),'successful_inverse_count':len(good),
                         'scope':'Unseen offset TCP targets, seeded nearby branch only; wire joint precision 0.001 deg'}
                for label in ['local','native']:
                    summary[label+'_max_position_mm']=max((c[label+'_independent_metrics']['position_mm']['max'] for c in good),default=None)
                    summary[label+'_max_orientation_deg']=max((c[label+'_independent_metrics']['orientation_deg']['max'] for c in good),default=None)
                gate=get_meta(db,'gate')
                summary['passed']=bool(len(good)==12 and summary['local_max_position_mm']<=gate['position_max_mm']
                                       and summary['local_max_orientation_deg']<=gate['orientation_max_deg'])
                candidate['offset_target_validation']=summary
                print(f"OFFSET TARGET CHECK {profile}: local={summary['local_max_position_mm']}mm, native={summary['native_max_position_mm']}mm, pass={summary['passed']}",flush=True)
            if identity(reader.snapshot(profiles))!=identity(snapshot):
                raise ValueError('Controller/context changed during offset validation')
        for profile,candidate in report['profiles'].items():
            db.execute('UPDATE candidates SET payload=? WHERE profile=?',(encoded(candidate),profile))
        put_meta(db,'fit_report',report);put_meta(db,'offset_validation_state','complete')
        export_report(Path(args.database).resolve().parent,report)
    except BaseException:
        put_meta(db,'offset_validation_state','incomplete');raise
    finally:
        db.close()


def export_report(directory, report):
    # Derived diagnostics, not configuration. No raw FTP credentials are retained.
    (directory / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    lines = ['# 汇川只读运动学辨识实验', '', f"- 型号：`{report['model']}`", f"- 固件：`{report['firmware']}`",
             f"- 模型方法：`{report.get('method','rigid')}`",
             '- 未触发运动，未修改控制器；候选模型未启用，未修改生产配置库。',
             '- 数值一致性实验，不代表实体定位精度。关节输入保留3位小数，正解输出保留6位小数。',
             '- 参数为等效固定变换，不解释成唯一的出厂机械参数。', '',
             '|工具/工件/负载|数据集|数量|原模型TCP最大误差mm|拟合TCP平均mm|P95 mm|最大mm|姿态最大deg|',
             '|---|---|---:|---:|---:|---:|---:|---:|']
    for profile, candidate in report['profiles'].items():
        for split, values in candidate['metrics'].items():
            m = values['fitted_tcp']; p = m['position_mm']
            lines.append(f"|{profile}|{split}|{m['count']}|{values['baseline_tcp']['position_mm']['max']:.6f}|{p['mean']:.6f}|{p['p95']:.6f}|{p['max']:.6f}|{m['orientation_deg']['max']:.6f}|")
    for profile, candidate in report['profiles'].items():
        validation = candidate['independent_controller_ik_validation']
        if isinstance(validation, dict):
            validation = f"最大TCP误差 {validation['max_position_mm']:.6f} mm / {validation['max_orientation_deg']:.6f} deg（12组近邻初值逆解，不是全局分支验收）"
        lines += ['', f"## {profile}", '', f"- 初步数值门限：{'通过' if candidate['preliminary_numerical_gate_passed'] else '未通过'}（最大0.05 mm / 0.01 deg）。",
                  '- 即使通过也不自动部署；不包含碰撞检查、全局逆解分支验收或实体精度验收。',
                  f"- 独立控制器逆解验证：{validation}", '']
        if 'offset_target_validation' in candidate:
            v=candidate['offset_target_validation']
            lines += [f"- 未见过的偏移TCP目标验证：{v['successful_inverse_count']}/{v['case_count']}组完成；",
                      f"  本地逆解再交控制器正解，最大误差 {v['local_max_position_mm']:.6f} mm / {v['local_max_orientation_deg']:.6f} deg；",
                      f"  对照控制器原生逆解闭环，最大误差 {v['native_max_position_mm']:.6f} mm / {v['native_max_orientation_deg']:.6f} deg。",
                      '  注意这里包含关节角按0.001 deg传输的舍入误差，不是机器人实际移动误差。', '']
    (directory / 'report.md').write_text('\n'.join(lines), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='action', required=True)
    c = subs.add_parser('collect'); c.set_defaults(func=collect)
    c.add_argument('--host', required=True); c.add_argument('--port', type=int, default=2222)
    c.add_argument('--ftp-port', type=int, default=7777); c.add_argument('--ftp-user', default='robot')
    c.add_argument('--rate', type=float, default=5); c.add_argument('--robot-label', required=True)
    c.add_argument('--profiles', nargs='+', default=['0,0,0', '1,1,0'])
    c.add_argument('--training-count', type=int, default=240); c.add_argument('--seed', type=int, default=20260906)
    c.add_argument('--output', required=True)
    f = subs.add_parser('fit'); f.set_defaults(func=fit); f.add_argument('--database', required=True)
    f.add_argument('--method', choices=['rigid', 'gravity'], default='rigid')
    v = subs.add_parser('validate-controller'); v.set_defaults(func=validate_controller)
    v.add_argument('--database', required=True); v.add_argument('--host', required=True)
    v.add_argument('--port', type=int, default=2222); v.add_argument('--ftp-port', type=int, default=7777)
    v.add_argument('--ftp-user', default='robot'); v.add_argument('--rate', type=float, default=5)
    o = subs.add_parser('validate-offsets'); o.set_defaults(func=validate_offsets)
    o.add_argument('--database', required=True); o.add_argument('--host', required=True)
    o.add_argument('--port', type=int, default=2222); o.add_argument('--ftp-port', type=int, default=7777)
    o.add_argument('--ftp-user', default='robot'); o.add_argument('--rate', type=float, default=5)
    args = parser.parse_args()
    try:
        args.func(args)
    except (OSError, ValueError, RuntimeError, sqlite3.Error) as error:
        print(f"FAILED CLOSED: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
