import Visualization
import Util
import Instances
import data_collection
import FB_solver

def display_instance(inst):
    Visualization.draw_grid_mapf_simulation(inst)

def solve_instance(inst,solvers = ["PR","FB"]):
    Util.solve_instance(solvers,inst)

def display_TEN(inst,delete = True, T = -1):
    V,S,D,E,obstacles = inst.V,inst.S,inst.D,inst.E,inst.obstacles
    if T == -1:
        P = FB_solver.fb_solver(V,S,D,E,obstacles)
        T = len(P[0])
    V_T,E_T = Util.create_time_expanded_network(T,V,S,D,E,obstacles,delete,delete)
    Visualization.draw_time_expanded_network(V_T,E_T)

def createFigure(integer):
    match integer:
        case 1:
            display_instance(Instances.generateGameInstance(2,5,trees=5,seed=20))
        case 2:
            Visualization.draw_simple_network()
        case 3:
            Visualization.draw_vertex_split_gadget()
        case 4:
            Visualization.draw_edge_collision_gadget()
        case 5:
            display_TEN(Instances.inst1,delete=False, T = 2)
        case 6:
            data_collection.reachability_analysis()
        case 7:
            display_instance(Instances.generateGrid(3,3,2,1,keep_edges_percent=1,trucks = 0,seed = 10))
        case 8:
            data_collection.generate_runtime_figure("PR","W","R",100,seed=18,trucks=0)
        case 9:
            data_collection.generate_runtime_figure("PR","H","R",100,seed=18,trucks=0)
        case 10:
            data_collection.generate_runtime_figure("FB","W","R",100,seed=18,trucks=0)
        case 11:
            data_collection.generate_runtime_figure("FB","H","R",100,seed=18,trucks=0)
        case 12:
            display_instance(Instances.generateDeliveryInstance(4,2,10,4,seed = 66))
        case 13:
            data_collection.generate_runtime_figure("PR","W","R",100,seed=18,trucks=0,log_scale=True,show_quadratic_ref=True,extended=True)
        case 14:
            data_collection.generate_runtime_figure("FB","W","R",100,seed=18,trucks=0,semi_log_scale=True)
        case _:
            print(f"Figure {integer} not found")
        
def createTable(integer):
    match integer:
        case 1:
            data_collection.compare_solvers("game",seed=28,test_size=100,cars=False)
        case 1:
            data_collection.compare_solvers("game",seed=28,test_size=100,cars=True)
        case 2:
            data_collection.compare_solvers("delivery",seed=18,test_size=100)
        case _:
            print(f"Table {integer} not found") 