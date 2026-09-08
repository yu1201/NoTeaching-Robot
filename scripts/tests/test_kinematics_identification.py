"""Offline tests only: never connect to a controller."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from inovance_kinematics_reference import allowed_request, numbers, InovanceReference
from kinematics_model_fit import (dh, exp_rotation, rotation_log, forward, fit_chain,
                                 error_metrics, inverse_local, pose_matrix, prepare_flexibility,
                                 fit_flexible_chain, evaluate_model)
from run_kinematics_identification import make_design, connect_db


class IdentificationTests(unittest.TestCase):
    def test_whitelist(self):
        for cmd in ['Motor ON', 'Prg Start', 'Set_Mode 2', 'UserLogin 2 secret', 'AcqPermit',
                    'Get_Unknown', 'Get_RobotType$$@@Motor ON', 'Get_ToolData 16',
                    'Get_RobJToRobP nan,0,0,0,0,0,0,0;0,0,0,0,0,0 0,0,0']:
            self.assertFalse(allowed_request(cmd), cmd)
        self.assertTrue(allowed_request('Get_RobJToRobP 1,2,3,4,5,6,0,0;0,0,0,0,0,0 0,0,0'))
        self.assertTrue(allowed_request('Get_RobPToRobJ 1,2,3,4,5,6;-1,0,1,1;0,0,0,0,0,0 1,1,0'))
        reader = InovanceReference('127.0.0.1')
        with self.assertRaises(ValueError):
            reader.query('Motor ON')  # rejected before any socket operation

    def test_reply_validation(self):
        for text in ['=1,nan', '=1,inf', '=1,2,3', '=1bad,2', 'ok', '=1,1e999']:
            with self.assertRaises(ValueError):
                numbers(text, 2)
        np.testing.assert_array_equal(numbers('=1,2;', 2), [1, 2])

    def test_rotation_log(self):
        for v in [[0, 0, 0], [.01, -.02, .03], [np.pi, 0, 0], [0, np.pi-1e-7, 0]]:
            r = exp_rotation(np.array(v))
            np.testing.assert_allclose(exp_rotation(rotation_log(r)), r, atol=1e-7)
        np.testing.assert_allclose(pose_matrix([0,0,0,90,0,0])[:3,:3],
                                   exp_rotation(np.array([0,0,np.pi/2])), atol=1e-12)

    def test_design_has_no_training_leakage(self):
        limits = np.array([[-170,170],[-155,80],[-80,160],[-190,190],[-180,180],[-450,450]])
        data = make_design(limits, [-6,47,-47,50,-5,104], training_count=80)
        train = {tuple(q) for label,q in data if label == 'train'}
        tests = {tuple(q) for label,q in data if label.startswith('test')}
        self.assertFalse(train & tests)
        lo,hi=limits[:,0]+8,limits[:,1]-8
        for label,q in data:
            if label == 'train':
                self.assertTrue(np.all(((q-lo)/(hi-lo))[[1,4]] <= .651))
            if label == 'test_region':
                self.assertTrue(np.any(((q-lo)/(hi-lo))[[1,4]] >= .719))
        self.assertEqual(data, make_design(limits, [-6,47,-47,50,-5,104], training_count=80))

    def test_candidate_db_isolation(self):
        with self.assertRaises(ValueError):
            connect_db(Path('Data/ConfigStore.db'), writable=True)

    def test_known_chain_recovery_and_inverse(self):
        nominal = np.array([dh(170,90,500,0),dh(840,0,0,90),dh(205,90,-10,0),
                            dh(0,-90,1037,0),dh(0,90,0,0),dh(0,0,75,0)])
        rng = np.random.default_rng(910)
        truth = rng.normal(0, .2, 42)
        q = rng.uniform(-80,80,(180,6))
        target = forward(q, nominal, truth)
        fitted, details = fit_chain(q[:120],target[:120],nominal)
        holdout = error_metrics(forward(q[120:],nominal,fitted),target[120:])
        self.assertLess(holdout['position_mm']['max'], .001)
        self.assertLess(holdout['orientation_deg']['max'], .0001)
        self.assertFalse(details['bound_hit'])
        solved, ok = inverse_local(target[125],q[125]+.2,nominal,fitted,np.array([[-180,180]]*6))
        self.assertTrue(ok)
        self.assertLess(error_metrics(forward(solved,nominal,fitted),target[125:126])['position_mm']['max'],1e-4)

    def test_static_compliance_recovery(self):
        nominal = np.array([dh(170,90,500,0),dh(840,0,0,90),dh(205,90,-10,0),
                            dh(0,-90,1037,0),dh(0,90,0,0),dh(0,0,75,0)])
        rng=np.random.default_rng(997);q=rng.uniform(-100,100,(240,6));geometry=rng.normal(0,.05,42)
        geometry[:6]=0  # Current supported experiment: vertical base / world -Z gravity.
        truth=prepare_flexibility(q[:160],nominal,geometry)
        truth['coefficients_mrad']=[rng.normal(0,.03,len(c)).tolist() for c in truth['coefficients_mrad']]
        target=evaluate_model(q,nominal,geometry,truth)
        fitted,flex,details=fit_flexible_chain(q[:160],target[:160],nominal)
        holdout=error_metrics(evaluate_model(q[160:],nominal,fitted,flex),target[160:])
        self.assertLess(holdout['position_mm']['max'],.002)
        self.assertLess(holdout['orientation_deg']['max'],.0002)
        self.assertFalse(details['bound_hit'])
        solved,ok=inverse_local(target[180],q[180]+.1,nominal,fitted,np.array([[-180,180]]*6),flexibility=flex)
        self.assertTrue(ok)
        self.assertLess(error_metrics(evaluate_model(solved,nominal,fitted,flex),target[180:181])['position_mm']['max'],1e-4)

    def test_unmodelled_error_is_not_hidden_by_more_iterations(self):
        nominal = np.array([dh(170,90,500,0),dh(840,0,0,90),dh(205,90,-10,0),
                            dh(0,-90,1037,0),dh(0,90,0,0),dh(0,0,75,0)])
        rng=np.random.default_rng(919);q=rng.uniform(-90,90,(200,6));target=forward(q,nominal)
        # A non-geometric, discontinuous error outside the candidate model class.
        target[:,0,3]+=np.where(q[:,4]>0,.5,-.5)
        p,flex,_=fit_flexible_chain(q[:120],target[:120],nominal)
        error=error_metrics(evaluate_model(q[120:],nominal,p,flex),target[120:])
        self.assertGreater(error['position_mm']['max'],.05)


if __name__ == '__main__':
    unittest.main()
