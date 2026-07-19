import Instances
import Figures

# Part 1:
# Here, instances can be solved or just displayed


# change parameters of an instance if wanted
inst_grid = Instances.generateGrid(3,3,2,1,keep_edges_percent=1,trucks = 0,seed = 10)
inst_grid_horizontal = Instances.generateGridHorizontalPaths(8,5,5,keep_edges_percent=0.8,trucks=0,seed=16)
inst_game = Instances.generateGameInstance(2,5,trees=5,seed=20)
inst_delivery = Instances.generateDeliveryInstance(4,2,10,4,seed = 66)
inst_geometric = Instances.generateRandomGeometric(50,0.2,10,5,seed = 16)
inst_created = Instances.inst1

# choose instance from above
inst = inst_game

# remove hastag to solve the instance
#Figures.solve_instance(inst,solvers = ["PR","FB"])

# remove hashtag to display the instance
#Figures.display_instance(inst)

# remove hashtag to display the TEN of the instance
#Figures.display_TEN(inst)


#__________________________________________________________________________________________________________


# Part 2:
# Here, each figure and table of the thesis can be generated
# Runtimes may differ based on hardware

# remove hashtag to create the wanted figure
#Figures.createFigure(1)

# remove hashtag to create the wanted table
Figures.createTable(3)

