# Research-PragyaSharma

Please refer to this research as indicated below:

1. Distributed Control - Comprehensive directory for MilCom 2023 publication. Pybullet exps directory contains device-cloud communication with state machine implementation + results for Adaptation of Control (PID @local, MPC @cloud). More details can be found within the directory.
2. CARLA + Pylot + Scenario Runner - Current (Jan '24) working directory.
    - These are all separate repositories under NESL. Replicate with Pylot docker container (UC Berkeley), pull Pylot (NESL) modified code from lf_prototype branch and scenario runner (NESL) modified code from custom_scenarios branch.
    - On three separate terminals run:
        - docker stop pylot2 && docker start pylot2 && nvidia-docker exec -i -t pylot2 /home/erdos/workspace/pylot/scripts/run_simulator.sh
        - nvidia-docker exec -i -t pylot2 /home/erdos/workspace/scenario_runner/run_scenario.sh Custom_1
        - rp (--> short for run program, internal command)
    - Running these commands will have the CARLA simulation start where the lead car is spawned 50 m ahead of the foller car. The view that you see is the follower's POV. To modify the lead car's behavior, edit the custom scenario file in the scenario runner repo.
  
Pending: Architectural diagram of interactions between Pylot, SR, and CARLA

