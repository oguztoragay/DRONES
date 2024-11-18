import localsolver
import sys


def read_instance(filename):
    # The input files follow the "Taillard" format
    with open(filename, 'r') as f:
        lines = f.readlines()

    first_line = lines[1].split()
    nb_jobs = int(first_line[0])
    nb_machines = int(first_line[1])

    # Processing times for each job on each machine
    # (given in the task order, the processing order is a decision variable)
    processing_times_task_order = [[int(proc_time) for proc_time in line.split()]
                                   for line in lines[3:3 + nb_jobs]]

    # Index of machines for each task
    machine_index = [[int(machine_i) - 1 for machine_i in line.split()]
                     for line in lines[4 + nb_jobs:4 + 2 * nb_jobs]]

    # Reorder processing times: processingTime[j][m] is the processing time of the
    # task of job j that is processed on machine m
    processing_times = [[processing_times_task_order[j][machine_index[j].index(m)]
                         for m in range(nb_machines)] for j in range(nb_jobs)]

    # Trivial upper bound for the start time of tasks
    max_start = sum(map(lambda processing_times_job: sum(processing_times_job), processing_times))

    return nb_jobs, nb_machines, processing_times, max_start


def main(instance_file, output_file, time_limit):
    nb_jobs, nb_machines, processing_times, max_start = read_instance(instance_file)

    with localsolver.LocalSolver() as ls:
        model = ls.model

        # Interval decisions: time range of each task
        # tasks[j][m] is the interval of time of the task of job j
        # which is processed on machine m
        tasks = [[model.interval(0, max_start) for _ in range(nb_machines)] for _ in range(nb_jobs)]

        # Task duration constraints
        for j in range(nb_jobs):
            for m in range(0, nb_machines):
                model.constraint(model.length(tasks[j][m]) == processing_times[j][m])

        # Create a LocalSolver array in order to be able to access it with "at" operators
        task_array = model.array(tasks)

        # List of the jobs on each machine
        jobs_order = [model.list(nb_jobs) for _ in range(nb_machines)]
        for m in range(nb_machines):
            # Each job is scheduled on every machine
            model.constraint(model.eq(model.count(jobs_order[m]), nb_jobs))

            # Every machine executes a single task at a time
            sequence_lambda = model.lambda_function(lambda i:
                model.at(task_array, jobs_order[m][i], m) < model.at(task_array, jobs_order[m][i + 1], m))
            model.constraint(model.and_(model.range(0, nb_jobs - 1), sequence_lambda))

        # List of the machines for each job
        machines_order = [model.list(nb_machines) for _ in range(nb_jobs)]
        for j in range(nb_jobs):
            # Every task is scheduled on its corresponding machine
            model.constraint(model.eq(model.count(machines_order[j]), nb_machines))

            # A job has a single task at a time
            sequence_lambda = model.lambda_function(lambda k:
                    model.at(task_array, j, machines_order[j][k]) < model.at(task_array, j, machines_order[j][k + 1]))
            model.constraint(model.and_(model.range(0, nb_machines - 1), sequence_lambda))

        # Minimize the makespan: the end of the last task
        makespan = model.max([model.end(model.at(task_array, j, m))
                             for j in range(nb_jobs) for m in range(nb_machines)])
        model.minimize(makespan)

        model.close()

        # Parametrize the solver
        ls.param.time_limit = time_limit
        ls.solve()

main('tai2020_5.txt', None, 3600)