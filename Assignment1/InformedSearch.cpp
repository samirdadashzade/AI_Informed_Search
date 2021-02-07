#include <cstdio>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <stdio.h>
#define MAX 100
#define INF 0x3F3F3F3F
using namespace std;

struct edge
{
	int node; double dist;
	edge(int node, double dist) : node(node), dist(dist) {}
};

bool operator < (edge a, edge b)
{
	return a.dist > b.dist;
}

int vertexId, squareId, from, to, start, dest; 
vector<vector<edge>> matrix;
double dist;
char sd;

//Dijkstra
vector<int> usedD;
vector<int> parentD;
vector<int> distancesD;
int fringeLookupD;

//A*
map<int, int> vertexes;
vector<int> usedA;
vector<int> parentA;
vector<int> gScores;
vector<int> fScores;
int fringeLookupA;


void loadData() {
	//read vertexes file as standart input
	freopen("v.txt", "r", stdin);

	//load vertexes
	while (scanf("%d,%d", &vertexId, &squareId) == 2)
	{
		vertexes[vertexId] = squareId;
	}

	//read squares file as standart input
	freopen("e.txt", "r", stdin);

	//resize the first level matrix
	matrix.resize(MAX);

	//load edges with weights
	while (scanf("%d,%d,%lf", &from, &to, &dist) == 3)
	{
		matrix[from].push_back(edge(to, dist));
		matrix[to].push_back(edge(from, dist));
	}
}

void printPath(vector<int>& parent, int current, int start)
{
	if (current != start)
	{
		printPath(parent, parent[current], start);
		printf("%d", current);

		if (current != dest)
		{
			printf("-");
		}
	} 
	else
	{
		printf("Path: %d-", current);
	}
}

void printDijkstraResult()
{
	printf("Dijkstra:\n");

	if (distancesD[dest] != INF) 
	{
		printf("Total path distance: %d\n", distancesD[dest]);
		printPath(parentD, dest, start);
		printf("\nNumber of fringe lookup: %d", fringeLookupD);
	}
	else
	{
		printf("Path not found\n");
	}
}

void printA_StarResult(double distance)
{
	printf("\n\nA*:\n");

	if (distance != -1)
	{
		printf("Total path distance: %f\n", distance);
		printPath(parentA, dest, start);
		printf("\nNumber of fringe lookup: %d", fringeLookupA);
	}
	else
	{
		printf("Path not found\n");
	}
}

double manhattanDistance(int v, int goal)
{
	//find square location of the vertexes
	int vSquare = vertexes.find(v)->second;
	int goalSquare = vertexes.find(goal)->second;

	//if vertexes are located inside the same square, the true cost h*(n) is 14.1
	//in order to make our heuristic to be admissable it has to be 0 <= h(n) <= h*(n)
	//return 0, as two points can overlap on the same point
	if (vSquare == goalSquare) return 0;

	//calculate the deviation of the vertexes from each other based on the x/y lines
	int xDistance = abs(goalSquare % 10 - vSquare % 10);
	int yDistance = abs(goalSquare / 10 - vSquare / 10);
	
	//calculate the Manhattan distance and return
	//if the vertices are not in the same square, 
	//in maximum they can be in the opposing side of different squares, so the true cost h*(n) is 
	//(xDistance + yDistance) * 100 + 28.2F.
	//That's why our heuristic is admissable
	return double((xDistance + yDistance) * 100);
}

void Dijkstra(vector<vector<edge>>& matrix, vector<int>& d, vector<int>& used, vector<int>& parent, int start)
{
	priority_queue<edge> pq;
	pq.push(edge(start, 0));

	d = vector<int>(MAX, INF);
	used = vector<int>(MAX, 0);
	parent = vector<int>(MAX, -1);

	d[start] = 0;
	used.push_back(start);

	while (!pq.empty())
	{
		fringeLookupD++;
		edge e = pq.top(); pq.pop();
		int v = e.node;

		if (e.dist > d[v]) continue;

		if (used[v] == 0) {
			for (int j = 0; j < matrix[v].size(); j++)
			{
				int to = matrix[v][j].node;
				int cost = matrix[v][j].dist;
				if (d[v] + cost < d[to])
				{
					d[to] = d[v] + cost;
					parent[to] = v;
					pq.push(edge(to, d[to]));
				}
			}
		}

		used[v] = 1;
	}
}

double A_Star(vector<vector<edge>>& matrix, vector<int>& gScores, vector<int>& fScores, vector<int>& used, vector<int>& parent, int start)
{
	priority_queue<edge> pq;
	parent = vector<int>(MAX, -1);

	gScores = vector<int>(MAX, INF);
	gScores[start] = 0;

	fScores = vector<int>(MAX, INF);
	fScores[start] = manhattanDistance(start, dest);
	pq.push(edge(start, fScores[start]));

	used = vector<int>(MAX, 0);
	used.push_back(start);

	while (!pq.empty())
	{
		fringeLookupA++;
		edge e = pq.top(); pq.pop();
		int v = e.node;

		if (v == dest) return e.dist;

		if (used[v] == 0) {
			for (int j = 0; j < matrix[v].size(); j++)
			{
				int to = matrix[v][j].node;
				int cost = matrix[v][j].dist;
				int gScore = cost + gScores[v];
				if (gScore < gScores[to])
				{
					gScores[to] = gScore;
					fScores[to] = gScore + manhattanDistance(to, dest);
					parent[to] = v;
					pq.push(edge(to, fScores[to]));
				}
			}
		}

		used[v] = 1;
	}

	return -1;
}


int main(void)
{
	start = 43;
	dest = 77;

	loadData();
	Dijkstra(matrix, distancesD, usedD, parentD, start);
	double distanceA = A_Star(matrix, gScores, fScores, usedA, parentA, start);

	printDijkstraResult();
	printA_StarResult(distanceA);

	printf("\n");
	return 0;
}
