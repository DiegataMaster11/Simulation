from machine import Pin, UART
from time import sleep
import itertools
from heapq import heappush, heappop
import network, socket, machine, time

#--------------------------------------------------------------------------------------------------------

# DIJKSTRA ALGORITHM PART

class Graph:
    def __init__(self, adjacency_list):
        self.adjacency_list = adjacency_list


class Vertex:
    def __init__(self, value):
        self.value = value


class Edge:
    def __init__(self, distance, vertex):
        self.distance = distance
        self.vertex = vertex


def dijkstra(graph, start, end):
    previous = {v: None for v in graph.adjacency_list.keys()}
    visited = {v: False for v in graph.adjacency_list.keys()}
    distances = {v: float("inf") for v in graph.adjacency_list.keys()}
    distances[start] = 0
    queue = PriorityQueue()
    queue.add_task(0, start)
    path = []
    while queue:
        removed_distance, removed = queue.pop_task()
        visited[removed] = True

        if removed is end:
            while previous[removed]:
                path.append(removed.value)
                removed = previous[removed]
            path.append(start.value)
            print(f"shortest distance to {end.value}: ", distances[end])
            print(f"path to {end.value}: ", path[::-1])
            return path[::-1], distances[end]

        for edge in graph.adjacency_list[removed]:
            if visited[edge.vertex]:
                continue
            new_distance = removed_distance + edge.distance
            if new_distance < distances[edge.vertex]:
                distances[edge.vertex] = new_distance
                previous[edge.vertex] = removed
                queue.add_task(new_distance, edge.vertex)
    return


# slightly modified heapq implementation from https://docs.python.org/3/library/heapq.html
class PriorityQueue:
    def __init__(self):
        self.pq = []  # list of entries arranged in a heap
        self.entry_finder = {}  # mapping of tasks to entries
        self.counter = itertools.count()  # unique sequence count

    def __len__(self):
        return len(self.pq)

    def add_task(self, priority, task):
        'Add a new task or update the priority of an existing task'
        if task in self.entry_finder:
            self.update_priority(priority, task)
            return self
        count = next(self.counter)
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heappush(self.pq, entry)

    def update_priority(self, priority, task):
        'Update the priority of a task in place'
        entry = self.entry_finder[task]
        count = next(self.counter)
        entry[0], entry[1] = priority, count

    def pop_task(self):
        'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.pq:
            priority, count, task = heappop(self.pq)
            del self.entry_finder[task]
            return priority, task
        raise KeyError('pop from an empty priority queue')


# testing the algorithm
vertices = [Vertex("A"), Vertex("B"), Vertex("C"), Vertex("D"), Vertex("E"), Vertex("F"), Vertex("G"), Vertex("H"), Vertex("I"), Vertex("J"), Vertex("K"), Vertex("L"), Vertex("M"), Vertex("N"), Vertex("Ñ"), Vertex("O"), Vertex("P"), Vertex("Q"), Vertex("R")]
A, B, C, D, E, F, G, H, I, J, K, L, M, N, Ñ, O, P, Q, R = vertices

adj_list = {
    A: [Edge(0.1, B), Edge(0.25, G)],
    B: [Edge(0.1, A), Edge(0.1, C)],
    C: [Edge(0.1, B), Edge(0.1, D)],
    D: [Edge(0.1, C), Edge(0.2, E)],
    E: [Edge(0.2, D), Edge(0.5, F), Edge(0.16671, H)],
    F: [Edge(0.5, E), Edge(0.1667, I)],
    G: [Edge(0.25, A), Edge(0.501, J), Edge(0.5, L)],
    H: [Edge(0.16671, E), Edge(0.501, I), Edge(0.0833, J)],
    I: [Edge(0.1667, F), Edge(0.501, H),  Edge(0.0833, K)],
    J: [Edge(0.501, G), Edge(0.0833, H), Edge(0.501, K), Edge(0.0833, M)],
    K: [Edge(0.501, J), Edge(0.0833, I), Edge(0.25, R)],
    L: [Edge(0.0833, G), Edge(0.501, M), Edge(0.1667, N)],
    M: [Edge(0.0833, J), Edge(0.501, L), Edge(0.16671, Ñ)],
    N: [Edge(0.5, Ñ), Edge(0.1667, L)],
    Ñ: [Edge(0.5, N), Edge(0.16671, M), Edge(0.2, O)],
    O: [Edge(0.2, Ñ), Edge(0.1, P)],
    P: [Edge(0.1, O), Edge(0.1, Q)],
    Q: [Edge(0.1, P), Edge(0.1, R)],
    R: [Edge(0.1, Q), Edge(0.25, K)],
}

my_graph = Graph(adj_list)

#dijkstra(my_graph, start=A, end=N)

A_coordinate = [0, 0]
B_coordinate = [0, 0.1]
C_coordinate = [0, 0.2]
D_coordinate = [0, 0.3]
E_coordinate = [0, 0.5]
F_coordinate = [0, 1]
G_coordinate = [2.25, 0]
H_coordinate = [0.166, 0.5]
I_coordinate = [0.166, 1]
J_coordinate = [0.25, 1]
K_coordinate = [0.25, 1]
L_coordinate = [0.33, 0]
M_coordinate = [0.33, 0.5]
N_coordinate = [0.5, 0]
Ñ_coordinate = [0.5, 0]
O_coordinate = [0.5, 0.7]
P_coordinate = [0.5, 0.8]
Q_coordinate = [0.5, 0.9]
R_coordinate = [0.5, 1]
#--------------------------------------------------------------------------------------------------------


# WIFI COMMUNICATION PART
SSID = 'TMNL-CCEA31'
PASSWORD = '3KJS848EE57RFP67'
WEBOTS_IP = '192.168.1.248'  
WEBOTS_PORT = 12346          
LOCAL_PORT = 12345    

# LEDS_AND_PINS
led_board = Pin(2, Pin.OUT)    
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)
# BUTTONS
button_left = Pin(16, Pin.IN, Pin.PULL_DOWN)

# CONNECT TO WI-FI
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
while not wlan.isconnected():
    time.sleep(0.2)
print('Connected to:', wlan.ifconfig())
server = socket.socket()
server.bind(('', LOCAL_PORT))
server.listen(1)
server.settimeout(0.1)

# ROBOT PATH
robot_path_msg, _ = dijkstra(my_graph, start=A, end=R)
print (robot_path_msg)
nodes_to_travel = len(robot_path_msg)
robot_path_string = "".join(robot_path_msg)

# OTHERS
msg = ""

while True:
    # RECEIVE DATA
    try:
        conn, addr = server.accept()
        data = conn.recv(1024)
        msg = data.decode().strip()
        print('Comando recibido:', msg)
    except:
        pass

    time.sleep(.5)
            
    if msg == "IDLE":
        led_blue.off()
        led_green.off()
        led_red.off()
        led_yellow.on()
        # SEND DATA
        try:
            s = socket.socket()
            s.connect((WEBOTS_IP, WEBOTS_PORT))
            s.send(robot_path_string + '\n')
            s.close()
        except Exception as e:
            print('Error al enviar mensaje a Webots:', e)
        time.sleep(.5)
        
    
    if len(msg) == 2:
        led_blue.on()
        led_green.off()
        led_red.off()
        led_yellow.off()
        
        # SEND DATA
        obstacles_nodes_str = str(msg)
        obstacles_nodes = list(obstacles_nodes_str)
        if obstacles_nodes[0] == "A":
            Node_1 = A
        elif obstacles_nodes[0] == "B":
            Node_1 = B
        elif obstacles_nodes[0] == "C":
            Node_1 = C
        elif obstacles_nodes[0] == "D":
            Node_1 = D
        elif obstacles_nodes[0] == "E":
            Node_1 = E
        elif obstacles_nodes[0] == "F":
            Node_1 = F
        elif obstacles_nodes[0] == "G":
            Node_1 = G
        elif obstacles_nodes[0] == "H":
            Node_1 = H
        elif obstacles_nodes[0] == "I":
            Node_1 = I
        elif obstacles_nodes[0] == "J":
            Node_1 = J
        elif obstacles_nodes[0] == "K":
            Node_1 = K
        elif obstacles_nodes[0] == "L":
            Node_1 = L
        elif obstacles_nodes[0] == "M":
            Node_1 = M
        elif obstacles_nodes[0] == "N":
            Node_1 = N
        elif obstacles_nodes[0] == "Ñ":
            Node_1 = Ñ
        elif obstacles_nodes[0] == "O":
            Node_1 = O
        elif obstacles_nodes[0] == "P":
            Node_1 = P
        elif obstacles_nodes[0] == "Q":
            Node_1 = Q
        
            
            
        if obstacles_nodes[1] == "A":
            Node_2 = A
        elif obstacles_nodes[1] == "B":
            Node_2 = B
        elif obstacles_nodes[1] == "C":
            Node_2 = C
        elif obstacles_nodes[1] == "D":
            Node_2 = D
        elif obstacles_nodes[1] == "E":
            Node_2 = E
        elif obstacles_nodes[1] == "F":
            Node_2 = F
        elif obstacles_nodes[1] == "G":
            Node_2 = G
        elif obstacles_nodes[1] == "H":
            Node_2 = H
        elif obstacles_nodes[1] == "I":
            Node_2 = I
        elif obstacles_nodes[1] == "J":
            Node_2 = J
        elif obstacles_nodes[1] == "K":
            Node_2 = K
        elif obstacles_nodes[1] == "L":
            Node_2 = L
        elif obstacles_nodes[1] == "M":
            Node_2 = M
        elif obstacles_nodes[1] == "N":
            Node_2 = N
        elif obstacles_nodes[1] == "Ñ":
            Node_2 = Ñ
        elif obstacles_nodes[1] == "O":
            Node_2 = O
        elif obstacles_nodes[1] == "P":
            Node_2 = P
        elif obstacles_nodes[1] == "Q":
            Node_2 = Q
            
            
        adj_list[Node_2] = [Edge(20, Node_1)]
        # ROBOT PATH
        robot_path_msg, _ = dijkstra(my_graph, start= Node_1, end= R)
        print (robot_path_msg)
        nodes_to_travel = len(robot_path_msg)
        
        robot_path_string = "".join(robot_path_msg)
        try:
            s = socket.socket()
            s.connect((WEBOTS_IP, WEBOTS_PORT))
            s.send(robot_path_string + '\n')
            s.close()
        except Exception as e:
            print('Error al enviar mensaje a Webots:', e)
        time.sleep(.5)
                


    sleep(0.02)