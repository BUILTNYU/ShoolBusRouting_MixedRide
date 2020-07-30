
from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from datetime import datetime
import pandas as pd
import numpy as np
import ast
from functools import partial
from six.moves import xrange


time_limit_seconds =  60*60
vehicle_distance = 1800000000000  # vehicle maximum travel distance in total

# Create and read data lists
time_matrix = [] 
pickups_deliveries = []
demand_list_SER = []
demand_list_SEW = []
distance_matrix = []

with open('./od_matrix_distance_Mixed_new.txt') as f:
    lines = f.read()
    distance_matrix = ast.literal_eval(lines)

with open('./od_matrix_duration_Mixed_new.txt') as f:
    lines = f.read()
    time_matrix = ast.literal_eval(lines)

with open('./PD_list_Mixed.txt') as f:
    lines = f.read()
    pickups_deliveries = ast.literal_eval(lines)

with open('./Demand_Re_Mixed.txt') as f:
    lines = f.read()
    demand_list_SER = ast.literal_eval(lines)

with open('./Demand_WC_Mixed.txt') as f:
    lines = f.read()
    demand_list_SEW = ast.literal_eval(lines)

# Create Data Moedl
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['time_matrix'] = time_matrix
    data['distance_matrix'] = distance_matrix
    data['pickups_deliveries'] = pickups_deliveries
    data['num_vehicles'] = 13
    data['vehicle_capacities_SER'] = [18,9,9,9,9,15,15,18,18,18,18,18,18]
    data['vehicle_capacities_SEW'] = [0,3,3,3,3,5,5,0,0,0,0,0,0]
    data['starts'] = [0,0,0,0,0,0,0,0,0,0,0,0,0]
    data['ends'] = [0,0,0,0,0,0,0,0,0,0,0,0,0]
    data['demands_SER'] = demand_list_SER # regular SE demand list
    data['demands_SEW'] = demand_list_SEW # wheel chair SE demand list

    return data

# Convert Index
def indexConvert(idx):
    if(idx is 0):
        return 0
    elif(idx in range(1, 57)):
        return 2
    elif(idx in range(84,115 )):
        return 3
    elif(idx in range(58, 83)):
        return 1
    else:
        return idx - 115 + 3

# Wirte results to excel
def save_as_excel(data, manager, routing, assignment):  # pylint:disable=too-many-locals
    """Prints assignment on console"""
    print('Objective: {}'.format(assignment.ObjectiveValue()))
    total_distance = 0
    total_load_SER = 0
    total_load_SEW = 0
    total_time = 0
    actualIndex = -1000
   
    outputFileName = './Output/Mixed_New_60min_output_new_format' + str(datetime.timestamp(datetime.now())) + '.csv'
    time_dimension = routing.GetDimensionOrDie('Time')
    with open(outputFileName, 'a') as out:
        out.write('RouteNumber,Mixed_Stop_ID,Sequence,SEW,SER,Time,Accumu_Time,Distance,Accumu_Distance\n')
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            distance_var = 0
            route_time = 0
            preivous_time_var = 0
            previous_distance_var = 0
            route_load_SER = 0
            route_load_SEW = 0
            previous_index = routing.Start(vehicle_id)
            sequence = 0
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                sequence = sequence + 1
                node_index = manager.IndexToNode(index)
                previous_distance_var = distance_var
                distance_var += routing.GetArcCostForVehicle(previous_index, index,
                                                        vehicle_id)

                route_load_SER += data['demands_SER'][node_index]
                route_load_SEW += data['demands_SEW'][node_index]

                if(actualIndex != indexConvert(node_index)):
                    actualIndex = indexConvert(node_index)                                                                          route_load_SEW)
                    plan_output = '{0},{1},{2},{3},{4},{5},{6},{7},{8}\n'.format(  vehicle_id,
                                                                           actualIndex,
                                                                           sequence,
                                                                           route_load_SEW,
                                                                           route_load_SER, 
                                                                           assignment.Min(time_var) - preivous_time_var,
                                                                           assignment.Min(time_var),
                                                                           distance_var - previous_distance_var,
                                                                           distance_var)
                    out.write(plan_output)
                preivous_time_var =  assignment.Min(time_var)

                previous_index = index
                print(previous_index)
                print(routing.NextVar(index))
                index = assignment.Value(routing.NextVar(index))
                print(index)

            print(index)
            actualidx2 = indexConvert(manager.IndexToNode(index)) 
            print(actualidx2)
            plan_output += ' {0} Load({1})\n'.format(actualidx2,
                                                     route_load_SER)
            plan_output += ' {0} Load({1})\n'.format(actualidx2,
                                                     route_load_SEW)
            plan_output += 'SER Load of the route: {}\n'.format(route_load_SER)
            plan_output += 'SEW Load of the route: {}\n'.format(route_load_SEW)
            plan_output += 'Time of the route: {}s\n'.format(
            assignment.Min(time_var))
            plan_output += 'Distance of the route: {}m\n'.format(
            distance_var)
            print(plan_output)
            total_time += assignment.Min(time_var)
            total_distance += distance_var
            total_load_SER += route_load_SER
            total_load_SEW += route_load_SEW
     
# # Wirte results to txt
def print_solution(data, manager, routing, assignment):  # pylint:disable=too-many-locals
    """Prints assignment on console"""
    print('Objective: {}'.format(assignment.ObjectiveValue()))
    total_distance = 0
    total_load_SER = 0
    total_load_SEW = 0
    total_time = 0
    actualIndex = -1000
    outputFileName = './Output/Mixed_New_60Min_output_new_format' + str(datetime.timestamp(datetime.now())) + '.txt'
    time_dimension = routing.GetDimensionOrDie('Time')
    with open(outputFileName, 'a') as out:

        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            distance_var = 0
            route_time = 0
            route_load_SER = 0
            route_load_SEW = 0
            previous_index = routing.Start(vehicle_id)

            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                node_index = manager.IndexToNode(index)
                distance_var += routing.GetArcCostForVehicle(previous_index, index,
                                                        vehicle_id)

                route_load_SER += data['demands_SER'][node_index]
                route_load_SEW += data['demands_SEW'][node_index]

                if(actualIndex != indexConvert(node_index)):
                    actualIndex = indexConvert(node_index)
                    plan_output += ' {0} SER Load({1})  SEW Load({5}) Time({2}, {3}) Distance({4},{4}) -> '.format(actualIndex, 
                                                                                                route_load_SER,
                                                                                                assignment.Min(time_var),
                                                                                                assignment.Max(time_var),
                                                                                                distance_var,
                                                                                                route_load_SEW)
                previous_index = index
                print(previous_index)
                print(routing.NextVar(index))
                index = assignment.Value(routing.NextVar(index))
                print(index)
            print(index)
            actualidx2 = indexConvert(manager.IndexToNode(index)) 
            print(actualidx2)
            plan_output += ' {0} Load({1})\n'.format(actualidx2,
                                                     route_load_SER)
            plan_output += ' {0} Load({1})\n'.format(actualidx2,
                                                     route_load_SEW)
            plan_output += 'SER Load of the route: {}\n'.format(route_load_SER)
            plan_output += 'SEW Load of the route: {}\n'.format(route_load_SEW)
            plan_output += 'Time of the route: {}s\n'.format(
            assignment.Min(time_var))
            plan_output += 'Distance of the route: {}m\n'.format(
            distance_var)
            print(plan_output)
            out.write(plan_output)
            total_time += assignment.Min(time_var)
            total_distance += distance_var
            total_load_SER += route_load_SER
            total_load_SEW += route_load_SEW
        print('Total time of all routes: {}s \n'.format(total_time))
        out.write('Total time of all routes: {}s \n'.format(total_time))
        print('Total distance of all routes: {}m'.format(total_distance))
        out.write('Total distance of all routes: {}m \n'.format(total_distance))
        print('Total SER load of all routes: {} \n'.format(total_load_SER))
        out.write('Total SER load of all routes: {} \n'.format(total_load_SER))
        print('Total SEW load of all routes: {} \n'.format(total_load_SEW))
        out.write('Total SEW load of all routes: {} \n'.format(total_load_SEW))

# Create time evaluator to return travel time
def create_time_evaluator(data):
    # """Creates callback to get total times between locations."""

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

    # Add Time constraint.
def add_time_window_constraints(routing, manager, data, time_evaluator):
    time = 'Time'
    routing.AddDimension(
        time_evaluator,
        0,  # allow waiting time
        5400,  # maximum time per vehicle, unit:s
        True,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except

# main function
def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['starts'],
                                           data['ends'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        # print(from_index, to_index)

        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        if(to_index == from_index and from_index == 0 ):
            return 0
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # print(from_index, to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    #Define cost of each arc.
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

    # Add Capacity constraint.
    # Standard seat capacity constraint
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
#         print(from_index)
        from_node = manager.IndexToNode(from_index)
        return data['demands_SER'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities_SER'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity_SER')

     # wheelchair capacity constraint
    def demand_callback_SEW(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
#         print(from_index)
        from_node = manager.IndexToNode(from_index)
        return data['demands_SEW'][from_node]

    demand_callback_SEW_index = routing.RegisterUnaryTransitCallback(
        demand_callback_SEW)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_SEW_index,
        0,  # null capacity slack
        data['vehicle_capacities_SEW'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity_SEW')
    
        # Add Time constraint
    time_evaluator_index = routing.RegisterTransitCallback(
        partial(create_time_evaluator(data), manager))
    add_time_window_constraints(routing, manager, data, time_evaluator_index)
    create_time_evaluator(data)
    
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
        save_as_excel(data, manager, routing, assignment)
    # print(assignment)

if __name__ == '__main__':
    main()






