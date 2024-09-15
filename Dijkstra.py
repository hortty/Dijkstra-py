from collections import defaultdict
import heapq

class Grafo:
    def __init__(self):
        self.dicionario_adj = defaultdict(list)

    def tem_aresta(self, u, v):
        return any(vert == v for vert, _ in self.dicionario_adj[u])
    
    def adiciona_aresta(self, u, v, weig): # como não é direcionado, adiciona aos dois nodes
        self.dicionario_adj[u].append((v, weig))
        self.dicionario_adj[v].append((u, weig)) 
        
    def adiciona_vertice(self, new):
        if new not in self.dicionario_adj:
            self.dicionario_adj[new] = []

    def get_peso(self, u, v): # peso da aresta entre os vértices
        for vert, weig in self.dicionario_adj[u]:
            if vert == v:
                return weig
        return None

    def grau(self, u): # grau do vertice
        return self.grau_entrada(u) + self.grau_saida(u)

    def imprime_lista_adjacencias(self):
        print("Lista de adjacências:")
        for vertice, vizinhos in self.dicionario_adj.items():
            print(f"{vertice}: {vizinhos}")
        
    def grau_entrada(self, u):  # arestas que chegam
        return len([vert for vert, _ in self.dicionario_adj.items() if vert == u])

    def grau_saida(self, u):
        return len(self.dicionario_adj[u])

    def remove_aresta(self, u, v):
        self.dicionario_adj[u] = [(vert, weig) for vert, weig in self.dicionario_adj[u] if vert != v]
        self.dicionario_adj[v] = [(vert, weig) for vert, weig in self.dicionario_adj[v] if vert != u]

    def remove_vertice(self, u):
        del self.dicionario_adj[u]
        for vert in self.dicionario_adj:
            self.dicionario_adj[vert] = [(v, weig) for v, weig in self.dicionario_adj[vert] if v != u]

    def Dijkstra(self, origem, destino):
        distancias = {v: float('inf') for v in self.dicionario_adj}
        predecessores = {v: None for v in self.dicionario_adj}
        distancias[origem] = 0
        fila = [(0, origem)]

        while fila:
            distancia_atual, vertice_atual = heapq.heappop(fila)

            if distancia_atual > distancias[vertice_atual]:
                continue

            for vizinho, peso in self.dicionario_adj[vertice_atual]:
                nova_distancia = distancia_atual + peso
                if nova_distancia < distancias[vizinho]:
                    distancias[vizinho] = nova_distancia
                    predecessores[vizinho] = vertice_atual
                    heapq.heappush(fila, (nova_distancia, vizinho))

        caminho = []
        custo_total = distancias[destino]
        vertice_atual = destino
        while vertice_atual is not None:
            caminho.append(vertice_atual)
            vertice_atual = predecessores[vertice_atual]
        caminho.reverse()
        
        return caminho, custo_total

grafo = Grafo()
grafo.adiciona_aresta('A', 'B', 1)
grafo.adiciona_aresta('A', 'C', 4)
grafo.adiciona_aresta('B', 'C', 2)
grafo.adiciona_aresta('B', 'D', 5)
grafo.adiciona_aresta('C', 'D', 1)
grafo.adiciona_aresta('D', 'C', 2)
grafo.adiciona_aresta('A', 'E', 10)
grafo.adiciona_aresta('B', 'E', 8)
grafo.adiciona_aresta('A', 'D', 4)
grafo.adiciona_aresta('E', 'D', 1)

origem = 'B'
destino = 'E'
caminho_mais_curto, custo_total = grafo.Dijkstra(origem, destino)
print(f"Caminho mais curto entre {origem} e {destino}: {caminho_mais_curto}")
print(f"Custo total do caminho: {custo_total}")
