class Graph {
    constructor() {
        this.adjList = new Map();
    }

    addNode(node) {
        if (!this.adjList.has(node)) {
            this.adjList.set(node, []);
        }
    }

    addEdge(nodeA, nodeB, distance) {
        if (nodeA === nodeB) {
            alert('Cannot add an edge with the same node.');
            return;
        }

        this.addNode(nodeA);
        this.addNode(nodeB);
        
        if (this.adjList.get(nodeA).find(edge => edge.node === nodeB)) {
            alert('Edge already exists.');
            return;
        }
        this.adjList.get(nodeA).push({ node: nodeB, distance: distance });
        this.adjList.get(nodeB).push({ node: nodeA, distance: distance });
        visualizeGraph();
    }

    deleteNode(node) {
        if (this.adjList.has(node)) {
            this.adjList.delete(node);
            for (let [key, value] of this.adjList) {
                this.adjList.set(key, value.filter(edge => edge.node !== node));
            }
            visualizeGraph();
        } else {
            alert('Node does not exist.');
        }
    }

    changeDistance(nodeA, nodeB, newDistance) {
        if (this.adjList.has(nodeA) && this.adjList.has(nodeB)) {
            this.adjList.set(nodeA, this.adjList.get(nodeA).map(edge => {
                if (edge.node === nodeB) edge.distance = newDistance;
                return edge;
            }));
            this.adjList.set(nodeB, this.adjList.get(nodeB).map(edge => {
                if (edge.node === nodeA) edge.distance = newDistance;
                return edge;
            }));
            visualizeGraph();
        } else {
            alert('One or both nodes do not exist.');
        }
    }

    dijkstra(startNode, endNode) {
        if (!this.adjList.has(startNode) || !this.adjList.has(endNode)) {
            alert('One or both nodes do not exist.');
            return;
        }

        let distances = {};
        let visited = new Set();
        let pq = new MinHeap();

        for (let key of this.adjList.keys()) {
            distances[key] = Infinity;
        }
        distances[startNode] = 0;
        pq.insert(startNode, 0);

        while (!pq.isEmpty()) {
            let { node, distance } = pq.extractMin();
            visited.add(node);

            for (let edge of this.adjList.get(node)) {
                if (!visited.has(edge.node)) {
                    let newDistance = distance + edge.distance;
                    if (newDistance < distances[edge.node]) {
                        distances[edge.node] = newDistance;
                        pq.insert(edge.node, newDistance);
                    }
                }
            }
        }

        console.log('Dijkstra distances:', distances);
        alert(`Shortest distance from ${startNode} to ${endNode} is ${distances[endNode]}`);
    }

    bellmanFord(startNode) {
        let distances = {};
        let edges = [];

        for (let key of this.adjList.keys()) {
            distances[key] = Infinity;
            for (let edge of this.adjList.get(key)) {
                edges.push([key, edge.node, edge.distance]);
            }
        }
        distances[startNode] = 0;

        for (let i = 0; i < this.adjList.size - 1; i++) {
            for (let [u, v, weight] of edges) {
                if (distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                }
            }
        }

        for (let [u, v, weight] of edges) {
            if (distances[u] + weight < distances[v]) {
                console.log('Graph contains a negative-weight cycle');
                return;
            }
        }

        console.log('Bellman-Ford distances:', distances);
        alert(`Shortest distances from ${startNode}: ${JSON.stringify(distances)}`);
    }

    dfs(startNode) {
        let visited = new Set();
        let result = [];

        const dfsUtil = (node) => {
            visited.add(node);
            result.push(node);

            for (let edge of this.adjList.get(node)) {
                if (!visited.has(edge.node)) {
                    dfsUtil(edge.node);
                }
            }
        };

        dfsUtil(startNode);

        console.log('DFS traversal:', result);
        alert(`DFS traversal starting from node ${startNode}: ${result}`);
    }

    minimumSpanningTree(startNode) {
        if (!this.adjList.has(startNode)) {
            alert('Start node does not exist.');
            return;
        }

        let mst = new Set();
        let visited = new Set();
        let pq = new MinHeap();
        let totalWeight = 0;

        pq.insert(startNode, 0);

        while (!pq.isEmpty()) {
            let { node, distance } = pq.extractMin();

            if (visited.has(node)) continue;

            visited.add(node);
            mst.add(node);
            totalWeight += distance;

            for (let edge of this.adjList.get(node)) {
                if (!visited.has(edge.node)) {
                    pq.insert(edge.node, edge.distance);
                }
            }
        }

        console.log('Minimum Spanning Tree nodes:', Array.from(mst));
        console.log('Total weight of MST:', totalWeight);
        alert(`Minimum Spanning Tree nodes starting from node ${startNode}: ${Array.from(mst)}\nTotal weight: ${totalWeight}`);
    }
}

class MinHeap {
    constructor() {
        this.heap = [];
    }

    insert(node, distance) {
        this.heap.push({ node, distance });
        this.bubbleUp();
    }

    bubbleUp() {
        let index = this.heap.length - 1;
        while (index > 0) {
            let element = this.heap[index];
            let parentIndex = Math.floor((index - 1) / 2);
            let parent = this.heap[parentIndex];

            if (parent.distance <= element.distance) break;

            this.heap[index] = parent;
            this.heap[parentIndex] = element;
            index = parentIndex;
        }
    }

    extractMin() {
        const min = this.heap[0];
        const end = this.heap.pop();
        if (this.heap.length > 0) {
            this.heap[0] = end;
            this.sinkDown(0);
        }
        return min;
    }

    sinkDown(index) {
        let length = this.heap.length;
        let element = this.heap[index];
        let leftChild, rightChild;
        let leftChildIndex, rightChildIndex;

        while (true) {
            leftChildIndex = 2 * index + 1;
            rightChildIndex = 2 * index + 2;
            let swap = null;

            if (leftChildIndex < length) {
                leftChild = this.heap[leftChildIndex];
                if (leftChild.distance < element.distance) {
                    swap = leftChildIndex;
                }
            }
            if (rightChildIndex < length) {
                rightChild = this.heap[rightChildIndex];
                if (
                    (swap === null && rightChild.distance < element.distance) ||
                    (swap !== null && rightChild.distance < leftChild.distance)
                ) {
                    swap = rightChildIndex;
                }
            }
            if (swap === null) break;

            this.heap[index] = this.heap[swap];
            this.heap[swap] = element;
            index = swap;
        }
    }

    isEmpty() {
        return this.heap.length === 0;
    }
}

const graph = new Graph();

function addEdge() {
    const nodeA = parseInt(document.getElementById('nodeA').value);
    const nodeB = parseInt(document.getElementById('nodeB').value);
    const distance = parseInt(document.getElementById('distance').value);
    if (!isNaN(nodeA) && !isNaN(nodeB) && !isNaN(distance)) {
        graph.addEdge(nodeA, nodeB, distance);
    }
}

function deleteNode() {
    const node = parseInt(document.getElementById('nodeA').value);
    if (!isNaN(node)) {
        graph.deleteNode(node);
    }
}

function changeDistance() {
    const nodeA = parseInt(document.getElementById('nodeA').value);
    const nodeB = parseInt(document.getElementById('nodeB').value);
    const newDistance = parseInt(document.getElementById('distance').value);
    if (!isNaN(nodeA) && !isNaN(nodeB) && !isNaN(newDistance)) {
        graph.changeDistance(nodeA, nodeB, newDistance);
    }
}

function dijkstra() {
    const startNode = parseInt(document.getElementById('startNode').value);
    const endNode = parseInt(document.getElementById('endNode').value);
    if (!isNaN(startNode) && !isNaN(endNode)) {
        graph.dijkstra(startNode, endNode);
    }
}

function bellmanFord() {
    const startNode = parseInt(document.getElementById('startNode').value);
    if (!isNaN(startNode)) {
        graph.bellmanFord(startNode);
    }
}

function dfs() {
    const startNode = parseInt(document.getElementById('startNode').value);
    if (!isNaN(startNode)) {
        graph.dfs(startNode);
    }
}

function minimumSpanningTree() {
    const startNode = parseInt(document.getElementById('startNode').value);
    if (!isNaN(startNode)) {
        graph.minimumSpanningTree(startNode);
    }
}

function visualizeGraph() {
    const container = document.getElementById('graphContainer');
    container.innerHTML = '';
    const radius = 200;
    const centerX = container.offsetWidth / 2;
    const centerY = container.offsetHeight / 2;
    const nodes = Array.from(graph.adjList.keys());
    const angleIncrement = (2 * Math.PI) / nodes.length;
    const nodePositions = {};

    nodes.forEach((node, index) => {
        const angle = index * angleIncrement;
        const x = centerX + radius * Math.cos(angle);
        const y = centerY + radius * Math.sin(angle);
        nodePositions[node] = { x, y };

        const nodeElement = document.createElement('div');
        nodeElement.classList.add('node');
        nodeElement.style.left = `${x - 20}px`; // Adjust for node size
        nodeElement.style.top = `${y - 20}px`; // Adjust for node size
        nodeElement.textContent = node;
        container.appendChild(nodeElement);
    });

    for (let [node, edges] of graph.adjList) {
        const { x: x1, y: y1 } = nodePositions[node];
        edges.forEach(edge => {
            const { x: x2, y: y2 } = nodePositions[edge.node];

            const distanceElement = document.createElement('div');
            distanceElement.classList.add('distance');
            distanceElement.textContent = edge.distance;

            const midX = (x1 + x2) / 2;
            const midY = (y1 + y2) / 2;
            distanceElement.style.left = `${midX}px`;
            distanceElement.style.top = `${midY}px`;
            container.appendChild(distanceElement);

            const length = Math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);
            const angle = Math.atan2(y2 - y1, x2 - x1) * (180 / Math.PI);

            const edgeElement = document.createElement('div');
            edgeElement.classList.add('edge');
            edgeElement.style.width = `${length}px`;
            edgeElement.style.transform = `rotate(${angle}deg)`;
            edgeElement.style.left = `${x1}px`;
            edgeElement.style.top = `${y1}px`;

            container.appendChild(edgeElement);
        });
    }
}

window.addEventListener('resize', visualizeGraph);
