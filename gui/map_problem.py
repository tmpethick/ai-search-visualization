from tkinter import *
import sys
import os.path
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from search import *
from search import breadth_first_tree_search as bfts, depth_first_tree_search as dfts, \
    depth_first_graph_search as dfgs, breadth_first_search as bfs, uniform_cost_search as ucs, \
    astar_search as asts
from utils import Stack, FIFOQueue, PriorityQueue
from copy import deepcopy
from operator import itemgetter

from map_parser import map_parser

root = None
city_coord = {}
romania_problem = None
algo = None
start = None
goal = None
counter = -1
city_map = None
frontier = None
front = None
node = None
next_button = None
explored = None

# def map_edge_labels(graph, f):
#     for k, dist in graph.dict.items():
#         graph.dict[k] = {k: f(v) for k, v in dist.items()}

def map_node_labels(graph, f):
    graph.locations = {k: f(v) for k, v in graph.locations.items()}

def fit_to_canvas(graph, width, height):
    nodes = graph.locations.values()
    min_x = min(map(itemgetter(0), nodes))
    min_y = min(map(itemgetter(1), nodes))
    shift_x = -min_x + 40
    shift_y = -min_y + 20

    max_x = max(map(itemgetter(0), nodes))
    max_y = max(map(itemgetter(1), nodes))
    scale_x = (width - 250) / max_x
    scale_y = (height - 100) / max_y
    
    map_node_labels(map_graph, lambda p: (p[0] * scale_x + shift_x, p[1] * scale_y + shift_y))

width = 750
height = 670
map_filename = sys.argv[1] if len(sys.argv) > 1 else "data/cph.txt"
map_graph = map_parser(map_filename)
fit_to_canvas(map_graph, width, height)

# map_edge_labels(map_graph, lambda x: x * 50)

def create_map(root):
    '''
    This function draws out the required map.
    '''
    global city_map, start, goal
    map_graph_locations = map_graph.locations
    margin = 5
    city_map = Canvas(root, width=width, height=height)
    city_map.pack()

    # Since lines have to be drawn between particular points, we need to list
    # them separately
    for (city_from, dist) in map_graph.dict.items():
        for city_to in dist.keys():
            make_line(
                city_map,
                map_graph_locations[city_from][0],
                height -
                map_graph_locations[city_from][1],
                map_graph_locations[city_to][0],
                height -
                map_graph_locations[city_to][1],
                map_graph.get(city_from, city_to))

    for city in map_graph_locations.keys():
        make_rectangle(
            city_map,
            map_graph_locations[city][0],
            height -
            map_graph_locations[city][1],
            margin,
            city)

    make_legend(city_map)


def make_line(map, x0, y0, x1, y1, distance):
    '''
    This function draws out the lines joining various points.
    '''
    map.create_line(x0, y0, x1, y1)
    map.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=distance)


def make_rectangle(map, x0, y0, margin, city_name):
    '''
    This function draws out rectangles for various points.
    '''
    global city_coord
    rect = map.create_rectangle(
        x0 - margin,
        y0 - margin,
        x0 + margin,
        y0 + margin,
        fill="white")
    if "Bucharest" in city_name or "Pitesti" in city_name or "Lugoj" in city_name \
            or "Mehadia" in city_name or "Drobeta" in city_name:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=E)
    else:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=SE)
    city_coord.update({city_name: rect})


def make_legend(map):

    rect1 = map.create_rectangle(600, 100, 610, 110, fill="white")
    text1 = map.create_text(615, 105, anchor=W, text="Un-explored")

    rect2 = map.create_rectangle(600, 115, 610, 125, fill="orange")
    text2 = map.create_text(615, 120, anchor=W, text="Frontier")

    rect3 = map.create_rectangle(600, 130, 610, 140, fill="red")
    text3 = map.create_text(615, 135, anchor=W, text="Currently Exploring")

    rect4 = map.create_rectangle(600, 145, 610, 155, fill="grey")
    text4 = map.create_text(615, 150, anchor=W, text="Explored")

    rect5 = map.create_rectangle(600, 160, 610, 170, fill="dark green")
    text5 = map.create_text(615, 165, anchor=W, text="Final Solution")


def tree_search(problem):
    '''
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Don't worry about repeated paths to a state. [Figure 3.7]
    This function has been changed to make it suitable for the Tkinter GUI.
    '''
    global counter, frontier, node
    # print(counter)
    if counter == -1:
        frontier.append(Node(problem.initial))
        # print(frontier)
        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        # print(node)
        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            # print(node)
            return node
        frontier.extend(node.expand(problem))
        # print(frontier)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        # print(node)
        display_explored(node)
    return None


def graph_search(problem):
    '''
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Figure 3.7]
    This function has been changed to make it suitable for the Tkinter GUI.
    '''
    global counter, frontier, node, explored
    if counter == -1:
        frontier.append(Node(problem.initial))
        explored = set()
        # print("Frontier: "+str(frontier))
        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        # print("Current node: "+str(node))
        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)
        # print("Frontier: " + str(frontier))
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        # print("Explored node: "+str(node))
        display_explored(node)
    return None


def display_frontier(queue):
    '''
    This function marks the frontier nodes (orange) on the map.
    '''
    global city_map, city_coord
    qu = deepcopy(queue)
    while qu:
        node = qu.pop()
        for city in city_coord.keys():
            if node.state == city:
                city_map.itemconfig(city_coord[city], fill="orange")


def display_current(node):
    '''
    This function marks the currently exploring node (red) on the map.
    '''
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="red")


def display_explored(node):
    '''
    This function marks the already explored node (gray) on the map.
    '''
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="gray")


def display_final(cities):
    '''
    This function marks the final solution nodes (green) on the map.
    '''
    global city_map, city_coord
    for city in cities:
        city_map.itemconfig(city_coord[city], fill="green")


def breadth_first_tree_search(problem):
    """Search the shallowest nodes in the search tree first."""
    global frontier, counter
    if counter == -1:
        frontier = FIFOQueue()
    return tree_search(problem)


def depth_first_tree_search(problem):
    """Search the deepest nodes in the search tree first."""
    # This search algorithm might not work in case of repeated paths.
    global frontier, counter
    if counter == -1:
        frontier = Stack()
    return tree_search(problem)


def breadth_first_search(problem):
    """[Figure 3.11]"""
    global frontier, node, explored, counter
    if counter == -1:
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node
        frontier = FIFOQueue()
        frontier.append(node)
        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        display_current(node)
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def depth_first_graph_search(problem):
    """Search the deepest nodes in the search tree first."""
    global frontier, counter
    if counter == -1:
        frontier = Stack()
    return graph_search(problem)


def best_first_graph_search(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    global frontier, node, explored, counter

    if counter == -1:
        f = memoize(f, 'f')
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node
        frontier = PriorityQueue(min, f)
        frontier.append(node)
        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        display_current(node)
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def uniform_cost_search(problem):
    """[Figure 3.14]"""
    return best_first_graph_search(problem, lambda node: node.path_cost)


def astar_search(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))


# TODO:
# Remove redundant code.
# Make the interchangbility work between various algorithms at each step.
def on_click():
    '''
    This function defines the action of the 'Next' button.
    '''
    global algo, counter, next_button, romania_problem, start, goal
    romania_problem = GraphProblem(start.get(), goal.get(), map_graph)
    if "Breadth-First Tree Search" == algo.get():
        node = breadth_first_tree_search(romania_problem)
        if node is not None:
            final_path = bfts(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Depth-First Tree Search" == algo.get():
        node = depth_first_tree_search(romania_problem)
        if node is not None:
            final_path = dfts(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Breadth-First Search" == algo.get():
        node = breadth_first_search(romania_problem)
        if node is not None:
            final_path = bfs(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Depth-First Graph Search" == algo.get():
        node = depth_first_graph_search(romania_problem)
        if node is not None:
            final_path = dfgs(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Uniform Cost Search" == algo.get():
        node = uniform_cost_search(romania_problem)
        if node is not None:
            final_path = ucs(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "A* - Search" == algo.get():
        node = astar_search(romania_problem)
        if node is not None:
            final_path = asts(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1


def reset_map():
    global counter, city_coord, city_map, next_button
    counter = -1
    for city in city_coord.keys():
        city_map.itemconfig(city_coord[city], fill="white")
    next_button.config(state="normal")

# TODO: Add more search algorithms in the OptionMenu


def main():
    global algo, start, goal, next_button
    cities = list(sorted(map_graph.locations.keys()))
    root = Tk()
    root.title("Road Map of Romania")
    root.geometry("950x1150")
    algo = StringVar(root)
    start = StringVar(root)
    goal = StringVar(root)
    algo.set("Breadth-First Search")
    start.set(cities[0])
    goal.set(cities[-1])
    algorithm_menu = OptionMenu(
        root,
        algo, "Breadth-First Tree Search", "Depth-First Tree Search",
        "Breadth-First Search", "Depth-First Graph Search",
        "Uniform Cost Search", "A* - Search")
    Label(root, text="\n Search Algorithm").pack()
    algorithm_menu.pack()
    Label(root, text="\n Start City").pack()
    start_menu = OptionMenu(root, start, *cities)
    start_menu.pack()
    Label(root, text="\n Goal City").pack()
    goal_menu = OptionMenu(root, goal, *cities)
    goal_menu.pack()
    frame1 = Frame(root)
    next_button = Button(
        frame1,
        width=6,
        height=2,
        text="Next",
        command=on_click,
        padx=2,
        pady=2,
        relief=GROOVE)
    next_button.pack(side=RIGHT)
    reset_button = Button(
        frame1,
        width=6,
        height=2,
        text="Reset",
        command=reset_map,
        padx=2,
        pady=2,
        relief=GROOVE)
    reset_button.pack(side=RIGHT)
    frame1.pack(side=BOTTOM)
    create_map(root)
    root.mainloop()


if __name__ == "__main__":
    main()
