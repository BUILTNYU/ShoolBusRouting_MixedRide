#!/usr/bin/env python
# coding: utf-8

# ## SE Routing for two schools

# #### This notebook is for SE routing (MUTI-Capacities,time window version)
# #### Capacity Constrains
# #### Pickup and Deliveries

# In[1]:


from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from datetime import datetime
import pandas as pd
import numpy as np
import ast
from functools import partial
from six.moves import xrange

# In[2]:


#number_of_points = 115
time_limit_seconds = 60*60
vehicle_distance = 180000000000  # vehicle maximum travel distance in total


# ### 1. Create Data Model

# In[3]:


# All the data are copied from Step 3 create Data Model


# In[4]:


time_matrix = [] 
pickups_deliveries = []
demand_list = []
distance_matrix = []
with open('./od_matrix_distance_GE.txt') as f:
    lines = f.read()
    distance_matrix = ast.literal_eval(lines)

with open('./od_matrix_duration_GE.txt') as f:
    lines = f.read()
    time_matrix = ast.literal_eval(lines)

with open('./Demand_list_GE.txt') as f:
    lines = f.read()
    demand_list = ast.literal_eval(lines)

with open('./PD_list_GE.txt') as f:
    lines = f.read()
    pickups_deliveries = ast.literal_eval(lines)
# In[5]:


# pickups_deliveries =  [[65, 1], [66, 2], [67, 3], [68, 4], [69, 5], [70, 6], [71, 7], [72, 8], [73, 9], [74, 10], [75, 11], [76, 12], [77, 13], [78, 14], [79, 15], [80, 16], [81, 17], [82, 18], [83, 19], [84, 20], [85, 21], [86, 22], [87, 23], [88, 24], [89, 25], [90, 26], [91, 27], [92, 28], [93, 29], [94, 30], [95, 31], [96, 32], [97, 33], [98, 34], [99, 35], [100, 36], [101, 37], [102, 38], [103, 39], [104, 40], [105, 41], [106, 42], [107, 43], [108, 44], [109, 45], [110, 46], [111, 47], [112, 48], [113, 49], [114, 50], [115, 51], [116, 52], [117, 53], [118, 54], [119, 55], [120, 56], [121, 57], [122, 58], [123, 59], [124, 60], [125, 61], [126, 62], [127, 63], [128, 64]]
# # In[6]:

# f_demand = [0 for i in range(65)]
# s_demand = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]
# f_demand_w = [0 for i in range(65)]
# s_demand_w = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1]
# demand_list_SER = f_demand + s_demand #regular SE student travelling without wheelchair
# demand_list_SEW = f_demand_w + s_demand_w # wheel chair SE student
# In[7]:



def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['time_matrix'] = time_matrix
    data['distance_matrix'] = distance_matrix
    data['pickups_deliveries'] = pickups_deliveries
    data['num_vehicles'] = 7
    data['vehicle_capacities'] = [18,18,18,18,18,18,30]
    #data['vehicle_capacities_SEW'] = [18,18,18,18,18,30,30]
    data['starts'] = [0, 0, 0, 0,0,0,0]
    data['ends'] = [0,0,0,0,0,0,0] 
    data['demands'] = demand_list
    #data['demands_SEW'] = demand_list_SEW # wheel chair SE demand list
    
    return data


# In[8]:

def indexConvert(idx):
    if(idx is 0):
        return 0
    elif(idx in range(1, 27)): #15881 has 26 GE stops
        return 1
    elif(idx in range(27, 58)): # 20259 has 31 stops
        return 2
    else:
        return idx - 57 + 2


def print_solution(data, manager, routing, assignment):
    """Prints assignment on console."""
    total_time = 0
    total_load = 0
    total_distance = 0
    #total_load_SEW = 0
    actualIndex = -1000
    outputFileName = './Output/GE_New_60min_output_new_format' + str(datetime.timestamp(datetime.now())) + '.txt'
    time_dimension = routing.GetDimensionOrDie('Time')
    with open(outputFileName, 'a') as out:
        for vehicle_id in range(data['num_vehicles']):
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!" + str(vehicle_id))
            index = routing.Start(vehicle_id)
            print(index)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            print(plan_output)
            route_time = 0
            # route_distance = 0
            route_load = 0
            #route_load_SEW = 0
            previous_index = routing.Start(vehicle_id)
            distance_var = 0

            while not routing.IsEnd(index):
                print('in qhil loop')
                time_var = time_dimension.CumulVar(index)
                # plan_output += '{0} Time({1},{2} Distance({3}) -> '.format(
                #     manager.IndexToNode(index), assignment.Min(time_var),
                #     assignment.Max(time_var),
                #     distance_var)
                # index = assignment.Value(routing.NextVar(index))
                # time_var = time_dimension.CumulVar(index)
                print('in here')
                node_index = manager.IndexToNode(index)
                distance_var += distance_matrix[node_index][manager.IndexToNode(previous_index)]

                route_load += data['demands'][node_index]
                #route_load_SEW += data['demands_SEW'][node_index]
                if(actualIndex != indexConvert(node_index)):
                    actualIndex = indexConvert(node_index)
                    plan_output += ' {0} Load({1}) Time({2}, {3}) Distance({4},{4})-> '.format(actualIndex, 
                                                                                              route_load,
                                                                                              assignment.Min(time_var),assignment.Max(time_var),
                                                                                              int(distance_var))
                    #plan_output += ' {0} SEW Load({1}) Time({2}, {3}) -> '.format(actualIndex, route_load_SEW, assignment.Min(time_var),assignment.Max(time_var))
                previous_index = index
                print(previous_index)
                print(routing.NextVar(index))
                index = assignment.Value(routing.NextVar(index))
                print(index)

                # print(previous_index, index, routing.GetArcCostForVehicle(
                #     previous_index, index, vehicle_id))
            print(index)
            actualidx2 = indexConvert(manager.IndexToNode(index)) 
            print(actualidx2)
            plan_output += ' {0} Load({1})\n'.format(actualidx2,
                                                     route_load)
            #plan_output += ' {0} Load({1})\n'.format(actualidx2,
            #                                         route_load_SEW)
            plan_output += 'Load of the route: {}\n'.format(route_load)
            #plan_output += 'SEW Load of the route: {}\n'.format(route_load_SEW)
            plan_output += 'Time of the route: {}s\n'.format(
            assignment.Min(time_var))

            plan_output += 'Distance of the route: {}m\n'.format(
            int(distance_var))       
            
            print(plan_output)
            out.write(plan_output)
            total_time += assignment.Min(time_var)
            total_distance += distance_var
            # total_distance += route_distance
            total_load += route_load
            #total_load_SEW += route_load_SEW
        print('Total time of all routes: {}s\n'.format(total_time))
        out.write('Total time of all routes: {}s\n'.format(total_time))
        print('Total Distance of all routes: {}m\n'.format(int(total_distance)))
        out.write('Total distance of all routes: {}m\n'.format(int(total_distance)))
        # print('Total distance of all routes: {}m'.format(total_distance))
        # out.write('Total distance of all routes: {}m'.format(total_distance))
        print('Total load of all routes: {}\n'.format(total_load))
        out.write('Total load of all routes: {}\n'.format(total_load))
        #print('Total SEW load of all routes: {}'.format(total_load_SEW))
        #out.write('Total SEW load of all routes: {}'.format(total_load_SEW))

# In[ ]:
def create_time_evaluator(data):
      #  """Creates callback to get total times between locations."""

    # def service_time(data, node):
    #     return abs(data['demands'][node]) * data['time_per_demand_unit']

    def travel_time(data, from_node, to_node):
    #"""Gets the travel times between two locations."""
        if from_node == to_node:
            travel_time = 0
        else:
             travel_time = time_matrix[from_node][to_node]
        return travel_time

    _total_time = {}
     # precompute total time to have time callback in O(1)
    for from_node in xrange(len(data['time_matrix'])):
        _total_time[from_node] = {}
        for to_node in xrange(len(data['time_matrix'])):
            if from_node == to_node:
               _total_time[from_node][to_node] = 0
            else:
               _total_time[from_node][to_node] = travel_time(data, from_node, to_node)

    def time_evaluator(manager, from_node, to_node):
      #"""Returns the total time between the two nodes"""
        return _total_time[manager.IndexToNode(from_node)][manager.IndexToNode(
            to_node)]

    return time_evaluator

    # Add Time Windows constraint.
def add_time_window_constraints(routing, manager, data, time_evaluator):
    time = 'Time'
    routing.AddDimension(
        time_evaluator,
        0,  # allow waiting time
        5400,  # maximum time per vehicle
        True,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['starts'],
                                           data['ends'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    #Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        if(to_index == from_index and from_index == 0 ):
            return 0
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        vehicle_distance,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

#   # Create and register a transit callback.
#     def time_callback(from_index, to_index):
#         """Returns the travel time between the two nodes."""
#         # Convert from routing variable Index to time matrix NodeIndex.
#         from_node = manager.IndexToNode(from_index)
#         to_node = manager.IndexToNode(to_index)
#         return data['time_matrix'][from_node][to_node]

#     transit_callback_index = routing.RegisterTransitCallback(time_callback)

#     # Define cost of each arc.
#     routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
#         print(from_index)
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

#     def demand_callback_SEW(from_index):
#         """Returns the demand of the node."""
#         # Convert from routing variable Index to demands NodeIndex.
# #         print(from_index)
#         from_node = manager.IndexToNode(from_index)
#         return data['demands_SEW'][from_node]

#     demand_callback_SEW_index = routing.RegisterUnaryTransitCallback(
#         demand_callback_SEW)
#     routing.AddDimensionWithVehicleCapacity(
#         demand_callback_SEW_index,
#         0,  # null capacity slack
#         data['vehicle_capacities_SEW'],  # vehicle maximum capacities
#         True,  # start cumul to zero
#         'Capacity_SEW')

    # Add Time Windows constraint.

    time_evaluator_index = routing.RegisterTransitCallback(
        partial(create_time_evaluator(data), manager))
    add_time_window_constraints(routing, manager, data, time_evaluator_index)
    
    create_time_evaluator(data)
    # Add time window constraints for each location except depot.
    # for location_idx, time_window in enumerate(data['time_windows']):
    #     if location_idx == 0:
    #         continue
    #     index = manager.NodeToIndex(location_idx)
    #     time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # # Add time window constraints for each vehicle start node.
    # for vehicle_id in range(data['num_vehicles']):
    #     index = routing.Start(vehicle_id)
    #     time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
    #                                             data['time_windows'][0][1])

    # Instantiate route start and end times to produce feasible times.
    # for i in range(data['num_vehicles']):
    #     routing.AddVariableMinimizedByFinalizer(
    #         time_dimension.CumulVar(routing.Start(i)))
    #     routing.AddVariableMinimizedByFinalizer(
    #         time_dimension.CumulVar(routing.End(i)))
    
    # Add pickup and delivery
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(
                delivery_index))
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index) <=
            distance_dimension.CumulVar(delivery_index))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
    search_parameters.time_limit.seconds = time_limit_seconds

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    print(assignment)
    if assignment:
        print("in assignment")
        print_solution(data, manager, routing, assignment)
    # print(assignment)

if __name__ == '__main__':
    main()

# In[ ]:




