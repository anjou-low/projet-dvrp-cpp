import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import namedtuple

Node = namedtuple(
    "Node", ["id", "x", "y", "demand", "service_time", "is_depot", "available_time"]
)
TourAtom = namedtuple("TourAtom", ["node_id", "load", "end_of_service", "distance"])

colors = [
    "#1CE6FF",
    "#FF34FF",
    "#FF4A46",
    "#008941",
    "#006FA6",
    "#A30059",
    "#7A4900",
    "#0000A6",
    "#63FFAC",
    "#B79762",
    "#004D43",
    "#8FB0FF",
    "#997D87",
    "#5A0007",
    "#809693",
    "#1B4400",
    "#4FC601",
    "#3B5DFF",
    "#4A3B53",
    "#FF2F80",
    "#61615A",
    "#BA0900",
    "#6B7900",
    "#00C2A0",
    "#FFAA92",
    "#FF90C9",
    "#B903AA",
    "#D16100",
    "#DDEFFF",
    "#000035",
    "#7B4F4B",
    "#A1C299",
    "#300018",
    "#0AA6D8",
    "#013349",
    "#00846F",
    "#372101",
    "#FFB500",
    "#C2FFED",
    "#A079BF",
    "#CC0744",
    "#C0B9B2",
    "#C2FF99",
    "#001E09",
    "#00489C",
    "#6F0062",
    "#0CBD66",
    "#EEC3FF",
    "#456D75",
    "#B77B68",
    "#7A87A1",
    "#788D66",
    "#885578",
    "#FAD09F",
    "#FF8A9A",
    "#D157A0",
    "#BEC459",
    "#456648",
    "#0086ED",
    "#886F4C",
]


def read_problem_data(filename):

    nodes = []
    with open(filename, "r") as file:
        while True:
            try:
                atoms = file.readline().strip().split(",")
                id = int(atoms[0])
                x = float(atoms[1])
                y = float(atoms[2])
                demand = float(atoms[3])
                service_time = float(atoms[4])
                is_depot = int(atoms[5])
                available_time = float(atoms[6])
                node = Node(id, x, y, demand, service_time, is_depot, available_time)
                nodes.append(node)
            except Exception as e:
                print(e)
                break

    return nodes


def read_timeslice_data(filename):

    available_c_nodes_ids = []
    committed_c_nodes_ids = []
    tour = []

    with open(filename, "r") as file:
        available_c_nodes_ids = file.readline().strip().split(",")[:-1]
        available_c_nodes_ids = [int(node_id) for node_id in available_c_nodes_ids]
        committed_c_nodes_ids = file.readline().strip().split(",")[:-1]
        committed_c_nodes_ids = [int(node_id) for node_id in committed_c_nodes_ids]

        while True:
            try:
                atoms = file.readline().strip().split(",")
                node_id = int(atoms[0])
                load = float(atoms[1])
                end_of_service = float(atoms[2])
                distance = float(atoms[3])
                tour_atom = TourAtom(node_id, load, end_of_service, distance)
                tour.append(tour_atom)
            except Exception as e:
                print(e)
                break

    return (available_c_nodes_ids, committed_c_nodes_ids, tour)


def atomize_tour(tour):
    """
    From a tour (a list of TourAtom) returns a list of list where each list is the partial tour of a vehicle
    """
    partial_tours = []
    l_bound = 0

    for index in range(1, len(tour)):
        if tour[index].node_id > 100:
            partial_tours.append(tour[l_bound:index])
            l_bound = index

    partial_tours.append(tour[l_bound:])

    return partial_tours


def gen_plot(nodes, num_timeslice, available_c_nodes_ids, committed_c_nodes_ids, tour):

    partial_tours = atomize_tour(tour)

    X = list(map(lambda node: node.x, nodes))
    Y = list(map(lambda node: node.y, nodes))

    demands = list(map(lambda node: node.demand, nodes))
    min_demand = min(demands)
    max_demand = max(demands)
    scaled_demands = list(
        map(
            lambda demand: 100 * (demand - min_demand) / (max_demand - min_demand),
            demands,
        )
    )

    nodes_color = ["b" if node.id in available_c_nodes_ids else "r" for node in nodes]
    for committed_node_id in committed_c_nodes_ids:
        nodes_color[committed_node_id] = "g"

    plt.scatter(X, Y, scaled_demands, nodes_color)

    for partial_tour in partial_tours:
        vehicle_number = partial_tour[0].node_id - 100
        prev_x = X[0]
        prev_y = Y[0]

        for tour_atom in partial_tour[1:]:
            new_x = X[tour_atom.node_id]
            new_y = Y[tour_atom.node_id]

            plt.plot([prev_x, new_x], [prev_y, new_y], c=colors[vehicle_number])

            prev_x = new_x
            prev_y = new_y

    for index, pos in enumerate(zip(X, Y)):
        plt.text(pos[0], pos[1], str(index), color="black", fontsize=6)

    plt.title("Timeslice num : " + str(num_timeslice))

    plt.show()


fig = plt.figure()


def animate(i):

    nodes = read_problem_data("problem_data.txt")
    filename = "timeslice_" + str(i + 1) + ".txt"
    available_c_nodes_ids, committed_c_nodes_ids, tour = read_timeslice_data(filename)

    partial_tours = atomize_tour(tour)

    X = list(map(lambda node: node.x, nodes))
    Y = list(map(lambda node: node.y, nodes))

    demands = list(map(lambda node: node.demand, nodes))
    min_demand = min(demands)
    max_demand = max(demands)
    scaled_demands = list(
        map(
            lambda demand: 100 * (demand - min_demand) / (max_demand - min_demand),
            demands,
        )
    )

    nodes_color = ["b" if node.id in available_c_nodes_ids else "r" for node in nodes]
    for committed_node_id in committed_c_nodes_ids:
        nodes_color[committed_node_id] = "g"

    plt.scatter(X, Y, scaled_demands, nodes_color)

    for partial_tour in partial_tours:
        vehicle_number = partial_tour[0].node_id - 100
        prev_x = X[0]
        prev_y = Y[0]

        for tour_atom in partial_tour[1:]:
            new_x = X[tour_atom.node_id]
            new_y = Y[tour_atom.node_id]

            plt.plot([prev_x, new_x], [prev_y, new_y], c=colors[vehicle_number])

            prev_x = new_x
            prev_y = new_y


if __name__ == "__main__":

    NODE_SIZE = 100  # Used to compute the diameter of a node
    NUM_TIMESLICES = 38

    # FOR THE ANIMATION UNCOMENT THE TWO LINES BELOW
    # anim = FuncAnimation(fig, animate, frames=25)
    # plt.show()

    # AND COMMENT THE REST BELOW
    nodes = read_problem_data("problem_data.txt")
    timeslices_data = ["padding"]

    for i in range(NUM_TIMESLICES):
        filename = "timeslice_" + str(i + 1) + ".txt"
        available_c_nodes_ids, committed_c_nodes_ids, tour = read_timeslice_data(
            filename
        )
        timeslices_data.append((available_c_nodes_ids, committed_c_nodes_ids, tour))

    # available_c_nodes_ids, committed_c_nodes_ids, tour = read_timeslice_data(
    #    "timeslice_1.txt"
    # )
    # print(available_c_nodes_ids)
    # print(committed_c_nodes_ids)
    # gen_plot(nodes, available_c_nodes_ids, committed_c_nodes_ids, tour)
    gen_plot(nodes, 1, *timeslices_data[1])
    gen_plot(nodes, 2, *timeslices_data[2])
    gen_plot(nodes, 3, *timeslices_data[3])
    gen_plot(nodes, 4, *timeslices_data[4])
    gen_plot(nodes, 10, *timeslices_data[10])
    gen_plot(nodes, 15, *timeslices_data[15])
    gen_plot(nodes, 38, *timeslices_data[38])
    #gen_plot(nodes, 40, *timeslices_data[40])

    # for tour_atom in tour:
    #    print(tour_atom)

    # partial_tours = atomize_tour(tour)

    # X = list(map(lambda node: node.x, nodes))
    # Y = list(map(lambda node: node.y, nodes))

    # demands = list(map(lambda node: node.demand, nodes))
    # min_demand = min(demands)
    # max_demand = max(demands)
    # scaled_demands = list(
    #    map(
    #        lambda demand: 100 * (demand - min_demand) / (max_demand - min_demand),
    #        demands,
    #    )
    # )

    # nodes_color = ["b" if node.id in available_c_nodes_ids else "r" for node in nodes]
    # for committed_node_id in committed_c_nodes_ids:
    #    nodes_color[committed_node_id] = "g"

    # plt.scatter(X, Y, scaled_demands, nodes_color)

    # for partial_tour in partial_tours:
    #    vehicle_number = partial_tour[0].node_id - 100
    #    prev_x = X[0]
    #    prev_y = Y[0]

    #    for tour_atom in partial_tour[1:]:
    #        new_x = X[tour_atom.node_id]
    #        new_y = Y[tour_atom.node_id]

    #        plt.plot([prev_x, new_x], [prev_y, new_y], c=colors[vehicle_number])

    #        prev_x = new_x
    #        prev_y = new_y

    # plt.show()
