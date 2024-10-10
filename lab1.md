# zad 1.
Wykonanie komendy spowodowało stworzenie się w katalogu ~/tiago/src wielu pakietów odpowiadających za m.in gazebo, urdf, elementy robota tiago.

# zad 2.
Colcon build buduje pliki pakietów dzięki czemu są gotowe do wykonania. parametr --symlink-install powoduje, że po zmianie w kodzie i plikach pakietów nie trzeba ponownie wykonywać komendy colcon build ponieważ dzieje się ona automatycznie.

# zad 3.
Po wykonaniu zadania, nasz nowo wytworzony węzęł hellos_stero_node z nowego pakietu hello_stero wypisał w terminalu: "hello world hello_stero package", co tłumacząc na polski oznacza "witaj świecie hello_stero pakiet".

# zad 4.
Węzły udostępniające akcję /execute_trajectory to /move_group a węzły korzystające z tej akcji to //p/l/a/y/_/m/o/t/i/o/n/2_move_group_node

# zad 5.
Po zaplanowaniu i wykonaniu ruchi do wybranej konfiguracji docelowej nowy węzeł zaczął korzystać z tej akcji, tym węzłem jest /rviz.

# zad 6.
Servisy udostępniające węzły /move_group*:

/apply_planning_scene: moveit_msgs/srv/ApplyPlanningScene
/check_state_validity: moveit_msgs/srv/GetStateValidity
/clear_octomap: std_srvs/srv/Empty
/compute_cartesian_path: moveit_msgs/srv/GetCartesianPath
/compute_fk: moveit_msgs/srv/GetPositionFK
/compute_ik: moveit_msgs/srv/GetPositionIK
/get_planner_params: moveit_msgs/srv/GetPlannerParams
/load_map: moveit_msgs/srv/LoadMap
/move_group/describe_parameters: rcl_interfaces/srv/DescribeParameters
/move_group/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
/move_group/get_parameters: rcl_interfaces/srv/GetParameters
/move_group/get_type_description: type_description_interfaces/srv/GetTypeDescription
/move_group/list_parameters: rcl_interfaces/srv/ListParameters
/move_group/set_parameters: rcl_interfaces/srv/SetParameters
/move_group/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
/plan_kinematic_path: moveit_msgs/srv/GetMotionPlan
/query_planner_interface: moveit_msgs/srv/QueryPlannerInterfaces
/save_map: moveit_msgs/srv/SaveMap
/set_planner_params: moveit_msgs/srv/SetPlannerParams
/get_planning_scene: moveit_msgs/srv/GetPlanningScene
/move_group_private_107652528944000/describe_parameters: rcl_interfaces/srv/DescribeParameters
/move_group_private_107652528944000/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
/move_group_private_107652528944000/get_parameters: rcl_interfaces/srv/GetParameters
/move_group_private_107652528944000/get_type_description: type_description_interfaces/srv/GetTypeDescription
/move_group_private_107652528944000/list_parameters: rcl_interfaces/srv/ListParameters
/move_group_private_107652528944000/set_parameters: rcl_interfaces/srv/SetParameters
/move_group_private_107652528944000/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically

Akcje:

/execute_trajectory: moveit_msgs/action/ExecuteTrajectory
/move_action: moveit_msgs/action/MoveGroup

# zad 7.
Węzeł /moveit_simple_controller_manager zarządza ruchem stawami robota jak widać na rqt_graphie.

# zad 8.

Z listy interfejsów wartości zadanej wszystkie są dostępne. Dla ramienia pierwszego złącza używane są:

arm_1_joint/effort [available] [unclaimed]
arm_1_joint/position [available] [claimed]
arm_1_joint/velocity [available] [unclaimed]

Wśród interfejsów stanu dla ramienia pierwszego mamy:

arm_1_joint/effort
arm_1_joint/position
arm_1_joint/velocity

# zad 9.

arm_controller:command_start_arm_1_joint/position 

    -> command_interfaces:command_end_arm_1_joint/position

state_interfaces:state_start_arm_1_joint/velocity 

    -> joint_state_broadcaster:state_end_arm_1_joint/velocity

state_interfaces:state_start_arm_1_joint/effort 

    -> joint_state_broadcaster:state_end_arm_1_joint/effort

state_interfaces:state_start_arm_1_joint/position 

    -> joint_state_broadcaster:state_end_arm_1_joint/position
    -> arm_controller:state_end_arm_1_joint/position
                                                
# zad 10.

Dzięki programowi z pakietu moveit_project ręka robota zmienia swoją pozycję na zadaną w programie.

# zad 11.

Po uruchomieniu nowo stworzonego węzła, wziualizowany jest po pierwszym wciśnięciu pokazuje się nad robotem napis planning a następnie napis executing.


