# Importa las librerías necesarias
import osmnx as ox
from googlemaps import Client as GoogleMaps
import networkx as nx
import heapq

# Configura osmnx
ox.config(use_cache=True, log_console=True)

# Inicializa el cliente de Google Maps con tu clave API
gmaps = GoogleMaps('AIzaSyAwKtct_JoZN7fuWz0gnWh0aQecsZDFJZI') # Reemplaza con tu clave real

# Función para obtener coordenadas con Google Maps API
def get_location_coordinates(address):
    geocode_result = gmaps.geocode(address)
    lat = geocode_result[0]['geometry']['location']['lat']
    lng = geocode_result[0]['geometry']['location']['lng']
    return (lat, lng)

# Función heurística para A* (puedes ajustarla según necesites)
def heuristic(a, b, G):
    return nx.shortest_path_length(G, a, b, weight='length')

# Implementación básica del algoritmo A*
def a_star_search(G, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float("inf") for node in G.nodes}
    g_score[start] = 0
    f_score = {node: float("inf") for node in G.nodes}
    f_score[start] = heuristic(start, goal, G)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            total_path = [current]
            while current in came_from:
                current = came_from[current]
                total_path.append(current)
            total_path.reverse()
            return total_path

        for neighbor in G.neighbors(current):
            tentative_g_score = g_score[current] + nx.shortest_path_length(G, current, neighbor, weight='length')
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal, G)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return False
# Principal
if __name__ == "__main__":
    # Extrae el grafo de calles
    place_name = "Culiacán, Sinaloa, México"
    G = ox.graph_from_place(place_name, network_type='drive')

    # Direcciones de inicio y fin
    start_address = 'Rey Melchor 18, 6 de Enero, 80010 Culiacán Rosales, Sin.'
    end_address = 'Forum Culiacán Centro Comercial'

    # Obtiene las coordenadas
    start_lat_lng = get_location_coordinates(start_address)
    end_lat_lng = get_location_coordinates(end_address)

    # Encuentra los nodos más cercanos en el grafo para las coordenadas
    start_node = ox.distance.nearest_nodes(G, start_lat_lng[1], start_lat_lng[0])
    end_node = ox.distance.nearest_nodes(G, end_lat_lng[1], end_lat_lng[0])

    # Busca la ruta con A*
    route = a_star_search(G, start_node, end_node)

    # Visualiza la ruta
    if route:
        # Crea un subgrafo con los nodos que están a una distancia razonable de la ruta
        # Definir una distancia en metros para la búsqueda de nodos cercanos
        distance = 1000  # Ajustar según sea necesario
        # Obtener todos los nodos dentro de una 'distance' metros de nuestra ruta
        route_nodes = set(route)
        for node in route:
            for neighbor in nx.ego_graph(G, node, radius=distance, distance='length').nodes:
                route_nodes.add(neighbor)
        
        # Crear un subgrafo con solo los nodos cercanos a la ruta
        H = G.subgraph(route_nodes)

        # Visualización de la ruta en el subgrafo
        fig, ax = ox.plot_graph(H, node_size=0, edge_linewidth=0.5, show=False, close=False)
        ox.plot_graph_route(H, route, route_color='red', route_linewidth=2, orig_dest_node_size=100, node_color='w', ax=ax)
        
        # Ajustar los límites alrededor de la ruta para una mejor vista
        north, south, east, west = ox.bbox_from_points([start_lat_lng, end_lat_lng], dist=500)
        ax.set_xlim(west, east)
        ax.set_ylim(south, north)
        
        plt.show()
    else:
        print("No se encontró ruta")