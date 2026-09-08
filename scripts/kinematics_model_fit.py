"""Offline, brand-independent 6R equivalent-chain identification (NumPy only).

All lengths are mm, joint angles deg, rotation errors rad. This experiment does
not install a model in the application. Fixed-link corrections are an equivalent
geometry, NOT uniquely identified factory parameters (the chain has gauge freedoms).
"""
from __future__ import annotations

import math
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_rotation(v):
    angle = np.linalg.norm(v)
    k = skew(v)
    if angle < 1e-8:
        return np.eye(3) + k + .5 * k @ k
    return np.eye(3) + math.sin(angle) / angle * k + (1 - math.cos(angle)) / angle**2 * k @ k


def rotation_log(r):
    """Stable SO(3) logarithm; also handles independent large-angle validation."""
    r = np.asarray(r)
    flat = r.reshape(-1, 3, 3)
    trace = np.trace(flat, axis1=1, axis2=2)
    theta = np.arccos(np.clip((trace - 1) / 2, -1, 1))
    vee = np.stack([flat[:, 2, 1] - flat[:, 1, 2], flat[:, 0, 2] - flat[:, 2, 0],
                    flat[:, 1, 0] - flat[:, 0, 1]], axis=1)
    scale = np.full_like(theta, .5)
    mid = (theta > 1e-7) & (theta < math.pi - 1e-5)
    scale[mid] = theta[mid] / (2 * np.sin(theta[mid]))
    out = vee * scale[:, None]
    for i in np.flatnonzero(theta >= math.pi - 1e-5):
        _, vectors = np.linalg.eigh((flat[i] + flat[i].T) / 2)
        axis = vectors[:, -1]
        if axis @ vee[i] < 0:
            axis = -axis
        out[i] = theta[i] * axis
    return out.reshape(r.shape[:-2] + (3,))


def pose_matrix(pose):
    """Wire XYZ,A,B,C with rotation Rz(A)*Ry(B)*Rx(C), not Euler subtraction."""
    x, y, z, a, b, c = np.asarray(pose)[:6]
    a, b, c = np.deg2rad([a, b, c])
    ca, cb, cc = np.cos([a, b, c]); sa, sb, sc = np.sin([a, b, c])
    m = np.eye(4)
    m[:3, :3] = [[ca*cb, ca*sb*sc-sa*cc, ca*sb*cc+sa*sc],
                 [sa*cb, sa*sb*sc+ca*cc, sa*sb*cc-ca*sc], [-sb, cb*sc, cb*cc]]
    m[:3, 3] = [x, y, z]
    return m


def dh(a, alpha, d, theta):
    a_rad, t_rad = np.deg2rad([alpha, theta])
    ca, sa = np.cos(a_rad), np.sin(a_rad); ct, st = np.cos(t_rad), np.sin(t_rad)
    return np.array([[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st],
                     [0, sa, ca, d], [0, 0, 0, 1.]])


def corrections_to_links(nominal, parameters):
    p = np.asarray(parameters).reshape(7, 6)
    corrections = np.repeat(np.eye(4)[None], 7, axis=0)
    for i in range(7):
        corrections[i, :3, :3] = exp_rotation(p[i, 3:] / 1000.)  # mrad
        corrections[i, :3, 3] = p[i, :3]
    return corrections[0], np.asarray(nominal) @ corrections[1:]


def forward(joints, nominal, parameters=None):
    q = np.atleast_2d(joints)
    if q.shape[1] != 6 or not np.isfinite(q).all():
        raise ValueError("Expected finite Nx6 joint degrees")
    p = np.zeros(42) if parameters is None else parameters
    base, links = corrections_to_links(nominal, p)
    out = np.repeat(base[None], len(q), axis=0)
    for i in range(6):
        angle = np.deg2rad(q[:, i]); c, s = np.cos(angle), np.sin(angle)
        rz = np.repeat(np.eye(4)[None], len(q), axis=0)
        rz[:, 0, 0] = c; rz[:, 1, 1] = c; rz[:, 0, 1] = -s; rz[:, 1, 0] = s
        out = out @ rz @ links[i]
    return out


def residual(predicted, target, orientation_lever_mm=500.):
    rotation = predicted[:, :3, :3] @ np.transpose(target[:, :3, :3], (0, 2, 1))
    return np.concatenate([predicted[:, :3, 3] - target[:, :3, 3],
                           orientation_lever_mm * rotation_log(rotation)], axis=1)


def error_metrics(predicted, target):
    r = residual(predicted, target, 1.)
    pos = np.linalg.norm(r[:, :3], axis=1)
    rot = np.rad2deg(np.linalg.norm(r[:, 3:], axis=1))
    def stats(v):
        return {"mean": float(v.mean()), "rms": float(np.sqrt(np.mean(v*v))),
                "p95": float(np.percentile(v, 95)), "max": float(v.max())}
    return {"count": len(pos), "position_mm": stats(pos), "orientation_deg": stats(rot)}


def fit_chain(joints, target, nominal, max_iterations=35):
    """Damped, regularized least squares; singular directions stay near nominal.

    42 parameters have deliberate gauge freedoms. Ridge/minimum-norm updates avoid
    claiming unique physical calibration. Bound corrections to 20 mm / 20 mrad.
    Only training data is accepted here; test data cannot influence optimization.
    """
    if len(joints) < 50:
        raise ValueError("At least 50 training poses required")
    p = np.zeros(42); damping = .01; ridge = 1e-6; history = []
    def calc(x):
        return residual(forward(joints, nominal, x), target).ravel()
    r = calc(p)
    for iteration in range(max_iterations):
        jac = np.empty((len(r), 42)); step = 1e-3
        for k in range(42):
            perturbed = p.copy(); perturbed[k] += step
            jac[:, k] = (calc(perturbed) - r) / step
        h = jac.T @ jac / len(joints)
        g = jac.T @ r / len(joints) + ridge * p
        old_cost = float(r @ r / len(joints) + ridge * (p @ p))
        accepted = False
        for _ in range(10):
            delta = np.linalg.solve(h + (damping + ridge) * np.eye(42), -g)
            candidate = np.clip(p + delta, -20, 20)
            candidate_r = calc(candidate)
            cost = float(candidate_r @ candidate_r / len(joints) + ridge * (candidate @ candidate))
            if cost < old_cost:
                p, r = candidate, candidate_r; damping = max(damping / 3, 1e-9)
                accepted = True
                break
            damping *= 10
        history.append({"iteration": iteration, "cost": old_cost, "accepted": accepted})
        if not accepted or np.linalg.norm(delta) < 1e-7 or old_cost - cost < 1e-12:
            break
    singular = np.linalg.svd(jac, compute_uv=False)
    return p, {"iterations": history, "rank_at_1e-6_relative": int(np.sum(singular > singular[0]*1e-6)),
               "parameter_count": 42, "singular_values": singular.tolist(),
               "equivalent_geometry_not_unique_factory_parameters": True,
               "bound_hit": bool(np.max(np.abs(p)) >= 19.999),
               "max_translation_correction_mm": float(np.max(np.abs(p.reshape(7, 6)[:, :3]))),
               "max_rotation_correction_mrad": float(np.max(np.abs(p.reshape(7, 6)[:, 3:])))}


def inverse_local(target, seed, nominal, parameters, limits, max_iterations=60, flexibility=None):
    """Offline seeded IK of exactly the fitted FK; never sends anything to a robot."""
    q = np.array(seed, dtype=float)
    for _ in range(max_iterations):
        r = residual(evaluate_model(q, nominal, parameters, flexibility), target[None]).ravel()
        if np.linalg.norm(r[:3]) < 1e-5 and np.linalg.norm(r[3:]) < 1e-5:
            return q, True
        jac = np.empty((6, 6))
        for k in range(6):
            qq = q.copy(); qq[k] += 1e-4
            jac[:, k] = (residual(evaluate_model(qq, nominal, parameters, flexibility), target[None]).ravel() - r) / 1e-4
        delta = np.linalg.lstsq(np.vstack([jac, .001*np.eye(6)]),
                                np.concatenate([-r, np.zeros(6)]), rcond=1e-10)[0]
        norm = np.max(np.abs(delta))
        if norm > 5:
            delta *= 5 / norm
        q = np.clip(q + delta, limits[:, 0], limits[:, 1])
    return q, False


def gravity_features(joints, nominal, geometry):
    """Static-gravity moment features, linear in downstream mass/first moments.

    Columns for link i on joint j: z_j dot ((p_i-p_j) cross g),
    z_j dot (R_i[:,k] cross g). Distances m; gravity direction -Z.
    Coefficients absorb stiffness and units. They are equivalent compliance,
    not uniquely identifiable real masses, centres of mass or stiffnesses.
    """
    q = np.atleast_2d(joints); base, links = corrections_to_links(nominal, geometry)
    frame = np.repeat(base[None], len(q), axis=0)
    origins = []; axes = []; frames = []
    for i in range(6):
        origins.append(frame[:, :3, 3].copy()/1000.)
        axes.append(frame[:, :3, 2].copy())
        angle = np.deg2rad(q[:, i]); c, s = np.cos(angle), np.sin(angle)
        rz = np.repeat(np.eye(4)[None], len(q), axis=0)
        rz[:,0,0]=c;rz[:,1,1]=c;rz[:,0,1]=-s;rz[:,1,0]=s
        frame = frame @ rz @ links[i]; frames.append(frame.copy())
    gravity = np.array([0.,0.,-1.]); result = []
    for j in range(6):
        columns = []
        for i in range(j,6):
            lever = frames[i][:,:3,3]/1000.-origins[j]
            columns.append(np.sum(axes[j]*np.cross(lever,gravity),axis=1))
            for k in range(3):
                columns.append(np.sum(axes[j]*np.cross(frames[i][:,:3,k],gravity),axis=1))
        result.append(np.stack(columns,axis=1))
    return result


def prepare_flexibility(joints, nominal, geometry):
    features = gravity_features(joints,nominal,geometry); mappings = []
    for values in features:
        _, singular, vt = np.linalg.svd(values,full_matrices=False)
        keep = (singular > 1e-6) & (singular > singular[0]*1e-4)
        mapping = vt[keep].T * (math.sqrt(len(joints))/singular[keep])
        mappings.append(mapping.tolist())
    return {'schema':'static-gravity-equivalent-v1','geometry_for_features':np.asarray(geometry).tolist(),
            'mappings':mappings,'coefficients_mrad':[[0.]*len(m[0]) for m in mappings],
            'gravity_direction':[0,0,-1], 'unique_physical_parameters':False}


def flexibility_offsets(joints,nominal,model):
    features = gravity_features(joints,nominal,model['geometry_for_features'])
    offsets = []
    for values,mapping,coeff in zip(features,model['mappings'],model['coefficients_mrad']):
        offsets.append(values @ np.array(mapping) @ np.asarray(coeff))
    return np.stack(offsets,axis=1)


def evaluate_model(joints, nominal, parameters, flexibility=None):
    if flexibility is None:
        return forward(joints,nominal,parameters)
    q = np.atleast_2d(joints)
    offsets = flexibility_offsets(q,nominal,flexibility)
    return forward(q+np.rad2deg(offsets/1000.),nominal,parameters)


def fit_flexible_chain(joints,target,nominal):
    """Training-only equivalent rigid+static-compliance fit, no residual table."""
    geometry, rigid_details = fit_chain(joints,target,nominal)
    flex = prepare_flexibility(joints,nominal,geometry)
    ranks = [len(x) for x in flex['coefficients_mrad']]
    features = [v @ np.array(m) for v,m in zip(gravity_features(joints,nominal,geometry),flex['mappings'])]
    def unpack(x):
        at=42; parts=[]
        for rank in ranks:
            parts.append(x[at:at+rank]);at+=rank
        return parts
    def calc(x):
        offsets=np.stack([v@c for v,c in zip(features,unpack(x))],axis=1)
        predicted=forward(joints+np.rad2deg(offsets/1000.),nominal,x[:42])
        return residual(predicted,target).ravel()
    p=np.concatenate([geometry,np.zeros(sum(ranks))]);ridge=1e-8;damping=.01;history=[]
    r=calc(p);size=len(p)
    for iteration in range(40):
        jac=np.empty((len(r),size))
        for k in range(size):
            pp=p.copy();pp[k]+=1e-3;jac[:,k]=(calc(pp)-r)/1e-3
        h=jac.T@jac/len(joints);g=jac.T@r/len(joints)+ridge*p
        old=float(r@r/len(joints)+ridge*(p@p));accepted=False
        for _ in range(10):
            delta=np.linalg.solve(h+(damping+ridge)*np.eye(size),-g)
            pp=np.clip(p+delta,-20,20);rr=calc(pp);cost=float(rr@rr/len(joints)+ridge*(pp@pp))
            if cost<old:
                p,r=pp,rr;damping=max(damping/3,1e-10);accepted=True;break
            damping*=10
        history.append({'iteration':iteration,'cost':old,'accepted':accepted})
        if not accepted or np.linalg.norm(delta)<1e-7 or old-cost<1e-13:
            break
    flex['coefficients_mrad']=[c.tolist() for c in unpack(p)]
    singular=np.linalg.svd(jac,compute_uv=False)
    details={'iterations':history,'rigid_initialization':rigid_details,'parameter_count':size,
             'gravity_feature_ranks':ranks,'rank_at_1e-6_relative':int(np.sum(singular>singular[0]*1e-6)),
             'bound_hit':bool(np.max(np.abs(p))>=19.999),
             'max_training_deflection_mrad':float(np.max(np.abs(flexibility_offsets(joints,nominal,flex))))}
    return p[:42],flex,details
