import localsolver
import sys
import math


def read_elem(filename):
    with open('C:\\Users\\otoragay\\PycharmProjects\\DRONES\\Nonlinear\\coordChrist100.dat') as f:
        return [str(elem) for elem in f.read().split()]


def main(instance_file, str_time_limit, output_file):
    nb_customers, nb_trucks, truck_capacity, dist_matrix_data, nb_depots, \
        nb_depot_copies, nb_total_locations, demands_data, max_dist = read_input_multi_trip_vrp(instance_file)

    with localsolver.LocalSolver() as ls:

        #
        # Declare the optimization model
        #
        model = ls.model

        # Locations visited by each truck (customer or depot)
        # Add copies of the depots (so that they can be visited multiple times)
        # Add an extra fictive truck (who will visit every depot that will not be visited by real trucks)
        visit_orders = [model.list(nb_total_locations) for k in range(nb_trucks + 1)]

        # The fictive truck cannot visit customers
        for i in range(nb_customers):
            model.constraint((model.contains(visit_orders[nb_trucks], i)) == False)

        # All customers must be visited by exactly one truck
        model.constraint(model.partition(visit_orders))

        # Create LocalSolver arrays to be able to access them with an "at" operator
        demands = model.array(demands_data)
        dist_matrix = model.array(dist_matrix_data)

        # A truck is used if it visits at least one customer
        trucks_used = [(model.count(visit_orders[k]) > 0) for k in range(nb_trucks)]

        dist_routes = [None] * nb_trucks
        for k in range(nb_trucks):
            sequence = visit_orders[k]
            c = model.count(sequence)

            # Compute the quantity in the truck at each step
            route_quantity_lambda = model.lambda_function(lambda i, prev: \
                                                              model.iif(sequence[i] < nb_customers,
                                                                        prev + demands[sequence[i]], 0))
            route_quantity = model.array(model.range(0, c), route_quantity_lambda, 0)

            # Trucks cannot carry more than their capacity
            quantity_lambda = model.lambda_function(
                lambda i: route_quantity[i] <= truck_capacity)
            model.constraint(model.and_(model.range(0, c), quantity_lambda))

            # Distance traveled by each truck
            dist_lambda = model.lambda_function(lambda i:
                                                model.at(dist_matrix,
                                                         sequence[i - 1],
                                                         sequence[i]))
            dist_routes[k] = model.sum(model.range(1, c), dist_lambda) \
                             + model.iif(c > 0,
                                         model.at(dist_matrix, nb_customers, sequence[0]) + \
                                         model.at(dist_matrix, sequence[c - 1], nb_customers), \
                                         0)

            # Limit distance traveled
            model.constraint(dist_routes[k] <= max_dist)

        # Total distance traveled
        total_distance = model.sum(dist_routes)

        # Objective: minimize the distance traveled
        model.minimize(total_distance)

        model.close()

        # Parameterize the solver
        ls.param.time_limit = int(str_time_limit)

        ls.solve()

        # Write solution output
        if output_file != None:
            with open(output_file, 'w') as file:
                file.write("File name: %s; totalDistance = %d \n" % (instance_file, total_distance.value))
                for k in range(nb_trucks):
                    if trucks_used[k].value:
                        file.write("Truck %d : " % (k))
                        for customer in visit_orders[k].value:
                            file.write("%d" % (customer) if customer < nb_customers else "%d" % (
                                -(math.floor((customer - nb_customers) / nb_depot_copies) + 1)))
                            file.write(" ")
                        file.write("\n")


def read_input_multi_trip_vrp(filename):
    if filename.endswith(".dat"):
        return read_input_multi_trip_vrp_dat(filename)
    else:
        raise Exception("Unknown file format")


def read_input_multi_trip_vrp_dat(filename):
    file_it = iter(read_elem('coordChrist100.dat'))

    nb_customers = int(next(file_it))
    nb_depots = int(next(file_it))

    depots_x = [None] * nb_depots
    depots_y = [None] * nb_depots
    for i in range(nb_depots):
        depots_x[i] = int(next(file_it))
        depots_y[i] = int(next(file_it))

    customers_x = [None] * nb_customers
    customers_y = [None] * nb_customers
    for i in range(nb_customers):
        customers_x[i] = int(next(file_it))
        customers_y[i] = int(next(file_it))

    truck_capacity = int(next(file_it)) // 2

    # Skip depots capacity infos (not related to the problem)
    for i in range(nb_depots):
        next(file_it)

    demands_data = [None] * nb_customers
    for i in range(nb_customers):
        demands_data[i] = int(next(file_it))

    nb_depot_copies = 20

    nb_total_locations = nb_customers + nb_depots * nb_depot_copies

    max_dist = 400

    nb_trucks = 3

    dist_matrix_data = compute_distance_matrix(depots_x, depots_y, customers_x, customers_y, nb_depot_copies)

    return nb_customers, nb_trucks, truck_capacity, dist_matrix_data, nb_depots, \
        nb_depot_copies, nb_total_locations, demands_data, max_dist


# Compute the distance matrix
def compute_distance_matrix(depots_x, depots_y, customers_x, customers_y, nb_depot_copies):
    nb_customers = len(customers_x)
    nb_depots = len(depots_x)
    nb_total_locations = nb_customers + nb_depots * nb_depot_copies
    dist_matrix = [[0 for _ in range(nb_total_locations)] for _ in range(nb_total_locations)]
    for i in range(nb_customers):
        dist_matrix[i][i] = 0
        for j in range(i, nb_customers):
            dist = compute_dist(customers_x[i], customers_x[j], customers_y[i], customers_y[j])
            dist_matrix[i][j] = dist
            dist_matrix[j][i] = dist
        for d in range(nb_depots):
            dist = compute_dist(customers_x[i], depots_x[d], customers_y[i], depots_y[d])
            for c in range(nb_depot_copies):
                j = nb_customers + d * nb_depot_copies + c
                dist_matrix[i][j] = dist
                dist_matrix[j][i] = dist

    for i in range(nb_customers, nb_total_locations):
        for j in range(nb_customers, nb_total_locations):
            # Going from one depot to an other is never worth it
            dist_matrix[i][j] = 100000

    return dist_matrix


def compute_dist(xi, xj, yi, yj):
    exact_dist = math.sqrt(math.pow(xi - xj, 2) + math.pow(yi - yj, 2))
    return int(math.floor(exact_dist + 0.5))


main('coordChrist100.dat', '3600', None)