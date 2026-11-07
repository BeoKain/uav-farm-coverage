import copy,csv,json,math,os,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/uav_farm'));sys.path.insert(0,str(ROOT/'tools'))
from terrain_links import TerrainGrid,evaluate
from baseline_planner import POLICY,poses
from cruise_profile import make_profile,kinematics
from execution_supervisor import ExecutionSupervisor
from prepare_case import prepare

class GeometryTests(unittest.TestCase):
    def test_range_and_terrain_are_independent_conditions(self):
        flat=TerrainGrid([-100,100],[-100,100],{(x,y):0. for x in (-100,100) for y in (-100,100)})
        self.assertTrue(evaluate(flat,(0,0,7),(10,0,.65),28,5)['geometry_eligible'])
        self.assertEqual(evaluate(flat,(0,0,7),(35,0,.65),28,5)['state'],'out_of_range')
        hill=TerrainGrid([0,10,20],[0,10,20],{(x,y):(10. if (x,y)==(10,10) else 0.) for x in (0,10,20) for y in (0,10,20)})
        self.assertEqual(evaluate(hill,(0,10,7),(20,10,.65),28,5)['state'],'terrain_blocked')

    def test_profile_obeys_limits_and_has_stationary_endpoints(self):
        a=(-65.,0.,7.,1.);b=(-40.,15.,12.,1.1)
        policy=dict(POLICY,horizontal_speed_m_s=1.4)
        p=make_profile(a,b,policy);duration=p['duration_s']
        self.assertEqual(kinematics(p,0),(0.,0.,0.));self.assertEqual(kinematics(p,duration),(1.,0.,0.))
        samples=[kinematics(p,duration*i/200) for i in range(201)]
        self.assertTrue(all(x[0]<=y[0]+1e-12 for x,y in zip(samples,samples[1:])))
        for start,end in zip(poses(a),poses(b)):
            horizontal=math.hypot(end[0]-start[0],end[1]-start[1]);vertical=abs(end[2]-start[2])
            distance=math.dist(start,end)
            self.assertLessEqual(max(v for _,v,_ in samples)*horizontal,policy['horizontal_speed_m_s']+1e-9)
            self.assertLessEqual(max(v for _,v,_ in samples)*vertical,policy['vertical_speed_m_s']+1e-9)
            self.assertLessEqual(max(abs(acc) for _,_,acc in samples)*distance,policy['acceleration_m_s2']+1e-9)

class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.case=ROOT/'scenarios/seed_202'
        self.plan=json.loads((self.case/'config/fleet_drill.json').read_text())
        self.terrain=TerrainGrid.from_csv(self.case/'config/terrain.csv')
        self.sup=ExecutionSupervisor(self.plan,self.terrain)
        self.sup.index=next(i for i,s in enumerate(self.sup.segments) if s['kind']=='SERVICE')
        self.zero={u:(0.,0.,0.) for u in self.sup.ids}
    def test_no_credit_when_monitor_blocks_progress(self):
        sup=self.sup
        for i in range(1,15):sup.tick(.05,sup.targets(),self.zero,i*.05,15+i*.05,allow=False)
        self.assertEqual(sum(v['credited_s'] for v in sup.ledger.values()),0)
        for i in range(15,35):sup.tick(.05,sup.targets(),self.zero,i*.05,15+i*.05,allow=True)
        credit=sum(v['credited_s'] for v in sup.ledger.values());self.assertGreater(credit,0)
        sup.tick(.05,sup.targets(),self.zero,2.,17.,allow=False)
        self.assertEqual(sum(v['credited_s'] for v in sup.ledger.values()),credit)
    def test_collapsed_pair_is_rejected(self):
        positions=self.sup.targets();positions['UAV1']=positions['UAV2']
        with self.assertRaises(ValueError):self.sup.tick(.05,positions,self.zero,.05,15.05)

class DistributionDataTests(unittest.TestCase):
    def test_recorded_routes_and_windows_cover_each_sensor_once(self):
        for seed in (101,202,303,404,505):
            p=ROOT/f'data/replay/seed_{seed}';meta=json.loads((p/'metadata.json').read_text())
            with (p/'assignments.csv').open() as f:assignments=list(csv.DictReader(f))
            self.assertEqual(len(assignments),30);self.assertEqual(len({r['sensor'] for r in assignments}),30)
            for r in assignments:
                self.assertIn(int(r['collector']),(0,1,2))
                self.assertGreaterEqual(float(r['start_s']),0)
                self.assertGreater(float(r['end_s']),float(r['start_s']))
                self.assertLessEqual(float(r['end_s']),meta['mission_duration_s'])
            with (p/'trajectories.csv').open() as f:rows=list(csv.DictReader(f))
            for member in range(3):
                times=[float(r['time_s']) for r in rows if int(r['collector'])==member]
                self.assertEqual(times[0],0);self.assertAlmostEqual(times[-1],meta['mission_duration_s'])
                self.assertTrue(all(a<b for a,b in zip(times,times[1:])))
    def test_published_network_totals_are_consistent(self):
        with (ROOT/'results/tables/network_results.csv').open() as f:rows=list(csv.DictReader(f))
        self.assertEqual(len(rows),15)
        self.assertEqual(sum(int(r['bytes_received']) for r in rows),43264000)
        self.assertEqual(sum(int(r['complete_by_arm_deadline']) for r in rows),417)
    def test_disposable_case_works_outside_checkout_and_does_not_write_scenarios(self):
        source=ROOT/'scenarios/seed_202/config/fleet_drill.json';before=source.read_bytes()
        with tempfile.TemporaryDirectory(prefix='uav case relocation ') as td:
            case=prepare(202,Path(td)/'case')
            self.assertTrue((case/'scripts/run_execution_sim.py').is_file())
            self.assertTrue((case/'models/farm_terrain/model.sdf').is_file())
            self.assertEqual(list((case/'runtime').iterdir()),[])
            self.assertEqual((case/'config/fleet_drill.json').read_bytes(),before)
        self.assertEqual(source.read_bytes(),before)
        with self.assertRaises(ValueError):prepare(202,ROOT/'scenarios/forbidden')

if __name__=='__main__':unittest.main()
