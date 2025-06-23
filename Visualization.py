import matplotlib.pyplot as plt
import itertools
from heapq import heappush, heappop

# DIJKSTRA ALGORITHM
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

        # this piece of code is not part of the video, but it's useful to print the final path and distance
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

Path_Letters, _ = dijkstra(my_graph, start= A, end= Q)

nodes_to_travel = len(Path_Letters)

coordinates = {
    "A": [0, 0],
    "B": [0, 0.15],
    "C": [0, 0.30],
    "D": [0, 0.45],
    "E": [0, 0.75],
    "F": [0, 1.50],
    "G": [0.375, 0],
    "H": [0.225, 0.75],
    "I": [0.225, 1.5],
    "J": [0.375, 0.75],
    "K": [0.375, 1.5],
    "L": [0.525, 0],
    "M": [0.525, 0.75],
    "N": [0.75, 0],
    "Ñ": [0.75, 0.75],
    "O": [0.75, 1.05],
    "P": [0.75, 1.2],
    "Q": [0.75, 1.35],
    "R": [0.75, 1.5]
}    

# Rotación 90° antihoraria: (x, y) → (y, -x)
x_rotado = [point[1] for point in coordinates.values()]
y_rotado = [-point[0] for point in coordinates.values()]
labels = list(coordinates.keys())

# Configuración del gráfico
plt.figure(figsize=(10, 8))
plt.scatter(x_rotado, y_rotado, color='red', s=50)

# Etiquetas de los puntos
for i, label in enumerate(labels):
    plt.text(x_rotado[i], y_rotado[i], label, fontsize=10, ha='center', va='bottom')

# --- Grupos de puntos alineados (vertical/horizontal después de rotación) ---
vertical_groups = [
    ["A", "G", "L", "N"],  # Mismo X original (vertical en rotado)
    ["E", "J"],            # X original = 0 y 0.375 → Y rotado = -0.75
    ["F", "K", "R"]        # X original = 0 y 0.375 → Y rotado = -1.5
]

# Añade esto en la sección de grupos horizontales/verticales:
horizontal_groups = [
    ["A", "B", "C", "D", "E", "F"],  # Y=0 original
    ["G", "J", "K",], ["H", "I"],              # Y=0.375 original
    ["L", "M", "J"],                       # Y=0.525 original (¡esta línea ya estaba!)
    ["N", "Ñ", "O", "P", "Q", "R"],   # Y=0.75 original
    ["M", "Ñ"]                        # Nueva conexión: M (Y=0.75 rotado) ↔ Ñ (Y=0.75 rotado)
]

# El resto del código se mantiene igual...

# Dibujar TODAS las conexiones con líneas discontinuas (azules)
for group in vertical_groups + horizontal_groups:
    x_points = [coordinates[p][1] for p in group]
    y_points = [-coordinates[p][0] for p in group]
    plt.plot(x_points, y_points, 'b--', alpha=0.7, linewidth=1.2)  # 'b--' = línea discontinua azul

# Líneas de referencia
plt.grid(True, linestyle=':', alpha=0.3)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)

# Títulos y ejes
plt.title("Conexiones exactas con líneas discontinuas", fontsize=14)
plt.xlabel("Eje Y original → ahora horizontal", fontsize=10)
plt.ylabel("Eje X original → ahora vertical (invertido)", fontsize=10)

# Ajustar límites
plt.xlim(min(x_rotado) - 0.1, max(x_rotado) + 0.1)
plt.ylim(min(y_rotado) - 0.1, max(y_rotado) + 0.1)

plt.show()


coordinates = {
    "A": [0, 0],
    "B": [0, 0.15],
    "C": [0, 0.30],
    "D": [0, 0.45],
    "E": [0, 0.75],
    "F": [0, 1.50],
    "G": [0.375, 0],
    "H": [0.225, 0.75],
    "I": [0.225, 1.5],
    "J": [0.375, 0.75],
    "K": [0.375, 1.5],
    "L": [0.525, 0],
    "M": [0.525, 0.75],
    "N": [0.75, 0],
    "Ñ": [0.75, 0.75],
    "O": [0.75, 1.05],
    "P": [0.75, 1.2],
    "Q": [0.75, 1.35],
    "R": [0.75, 1.5]
}

# Rotación 90° antihoraria: (x, y) → (y, -x)
x_rotado = [point[1] for point in coordinates.values()]
y_rotado = [-point[0] for point in coordinates.values()]
labels = list(coordinates.keys())

# --- Lista de colores personalizados ---
# Por defecto, todos los puntos son rojos ("red"), excepto los que definamos aquí.
# ... (código anterior hasta la definición de point_colors)

# --- Colorizar los puntos del camino de Dijkstra (verde) ---
point_colors = ["red"] * len(labels)  # Todos rojos por defecto

# Cambiar a verde solo los puntos en Path_Letters
for i, label in enumerate(labels):
    if label in Path_Letters:
        point_colors[i] = "green"

# --- Resto del código de visualización ---
plt.figure(figsize=(10, 8))
plt.scatter(x_rotado, y_rotado, color=point_colors, s=50)  # Puntos coloreados

# Etiquetas (igual que antes)
for i, label in enumerate(labels):
    plt.text(x_rotado[i], y_rotado[i], label, fontsize=10, ha='center', va='bottom')

# Conexiones (líneas discontinuas azules)
for group in vertical_groups + horizontal_groups:
    x_points = [coordinates[p][1] for p in group]
    y_points = [-coordinates[p][0] for p in group]
    plt.plot(x_points, y_points, 'b--', alpha=0.7, linewidth=1.2)

# Líneas de referencia y títulos
plt.grid(True, linestyle=':', alpha=0.3)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.title(f"Ruta Dijkstra: {' -> '.join(Path_Letters)}", fontsize=14)  # Muestra la ruta en el título
plt.xlabel("Eje Y original → ahora horizontal", fontsize=10)
plt.ylabel("Eje X original → ahora vertical (invertido)", fontsize=10)

plt.xlim(min(x_rotado) - 0.1, max(x_rotado) + 0.1)
plt.ylim(min(y_rotado) - 0.1, max(y_rotado) + 0.1)

plt.show()