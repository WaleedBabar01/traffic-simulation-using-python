# traffic-simulation-using-python
#project traffic simulation using python
import time
import random
import heapq
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tkinter import Tk, Label, Button, Entry, Frame, messagebox

# Graph representation
traffic_graph = {
    1: {2: 0, 3: 0},
    2: {1: 0, 3: 0},
    3: {1: 0, 2: 0}
}

car_positions = []

# Dijkstra's algorithm to find the optimal lane
def dijkstra(graph, start):
    pq = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    while pq:
        current_distance, current_node = heapq.heappop(pq)
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))
    return min(distances, key=distances.get)

# Update traffic graph weights
def update_traffic_graph(graph):
    for lane in graph:
        for neighbor in graph[lane]:
            graph[lane][neighbor] = random.randint(1, 20)

# Initialize random car positions
def initialize_cars():
    global car_positions
    car_positions = [(random.choice(list(traffic_graph.keys())), random.choice(list(traffic_graph.keys()))) for _ in range(5)]

# Animate car movement
def animate(i):
    plt.clf()
    G = nx.DiGraph()
    for node, neighbors in traffic_graph.items():
        for neighbor, weight in neighbors.items():
            G.add_edge(node, neighbor, weight=weight)
    pos = nx.circular_layout(G)
    edge_labels = nx.get_edge_attributes(G, 'weight')

    # Define the current lane with green light (from control_traffic function)
    global green_lane
    edge_colors = []
    for u, v in G.edges():
        if u == green_lane or v == green_lane:
            edge_colors.append('green')  # Green for emergency lane
        else:
            edge_colors.append('red')  # Red for other lanes

    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=1000, font_size=10, edge_color=edge_colors)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    global car_positions
    for idx, (current, target) in enumerate(car_positions):
        next_target = random.choice(list(traffic_graph[current].keys()))
        car_positions[idx] = (target, next_target)
        car_x, car_y = pos[current]
        plt.scatter(car_x, car_y, color='red', s=100)  # Represent car as a red point

    plt.title("Traffic Graph with Cars")

# Non-blocking traffic light control
def control_traffic(lane, status_label):
    global green_lane
    green_lane = lane  # Set the green lane to the selected lane

    green_time = 10
    status_label.config(text=f"Green In Lane {lane} Active\nRed In Other Lanes Active")
    root.after(green_time * 1000, lambda: status_label.config(text=f"Lane {lane} Green Time: {green_time} seconds"))

# Main simulation start function
def start_simulation(emergency_lane_entry, status_label):
    try:
        emergency_lane = int(emergency_lane_entry.get())
    except ValueError:
        messagebox.showerror("Invalid input", "Enter a valid lane number (1, 2, 3 or 0 for none).")
        return

    update_traffic_graph(traffic_graph)
    initialize_cars()
    optimal_lane = emergency_lane if emergency_lane in [1, 2, 3] else dijkstra(traffic_graph, start=1)
    status_label.config(text=f"{'Emergency' if emergency_lane in [1, 2, 3] else 'Optimal'} Lane: {optimal_lane}")
    control_traffic(optimal_lane, status_label)
    ani = FuncAnimation(plt.gcf(), animate, interval=1000)
    plt.show()

# Create the main Tkinter window
root = Tk()
root.title("Traffic Control Simulation")
root.geometry("800x600")

# Frame for user controls
control_frame = Frame(root)
control_frame.pack(pady=20)

# Label and Entry for emergency lane input
Label(control_frame, text="Enter Emergency Lane (1, 2, 3 or 0 for none):").grid(row=0, column=0, padx=10)
emergency_lane_entry = Entry(control_frame)
emergency_lane_entry.grid(row=0, column=1, padx=10)

# Button to start the simulation
Button(control_frame, text="Start Simulation", command=lambda: start_simulation(emergency_lane_entry, status_label)).grid(row=1, columnspan=2, pady=10)

# Status Label to show the current state
status_label = Label(root, text="Traffic Light Status", font=("Arial", 14))
status_label.pack(pady=20)

# Initialize green_lane variable to track which lane is green
green_lane = 0

# Run the Tkinter event loop
root.mainloop()
