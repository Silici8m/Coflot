from .base_strategy import AllocatorStrategy
from .allocation_interface import AllocationDecision, AllocationAction
from mission_manager.core.mission import MissionState

class NaiveQueueStrategy(AllocatorStrategy):
    def allocate(self) -> list[AllocationDecision]:
        decisions = []
        
        all_missions = self.registry.get_missions_list()
        all_robots_dict = self.robot_pool.get_all_robots()
        
        busy_robot_ids = set()
        
        for m in all_missions:
            # Si un robot est assigné et que la mission est active
            if m.assigned_robot_id and m.state not in [MissionState.FINISHED, MissionState.FAILED]:
                busy_robot_ids.add(m.assigned_robot_id)
        
        # 3. Liste des robots disponibles (Ceux qui ne sont pas dans busy_robot_ids)
        available_robots = []
        for r_id, robot in all_robots_dict.items():
            if r_id not in busy_robot_ids:
                available_robots.append(robot)
        
        # 4. Récupération des missions à traiter (PENDING)
        pending_missions = self.registry.get_missions_by_state(MissionState.PENDING)
        
        # 5. Tri : Priorité > Date
        pending_missions.sort(
            key=lambda m: (
                -m.priority, 
                m.request_stamp.sec, 
                m.request_stamp.nanosec
            )
        )
        
        # 6. Allocation FIFO
        for mission in pending_missions:
            if not available_robots:
                break # Plus de robots libres
            
            # On prend le premier robot disponible
            robot = available_robots.pop(0)
            
            decision = AllocationDecision(
                action=AllocationAction.START,
                mission_id=mission.mission_id,
                robot_id=robot.robot_id,
                reason="Naive FIFO allocation"
            )
            decisions.append(decision)
            
        return decisions