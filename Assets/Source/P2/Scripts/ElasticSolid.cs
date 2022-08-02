using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

public class ElasticSolid : MonoBehaviour
{
    public class Node
    {
        public int id;

        public Vector3 pos;
        public Vector3 vel;
        public Vector3 force;

        public float mass;
        public float damping;

        public Fixer fixer; //guardo una referencia del fixer que afecta a este nodo en caso de tenerlo
        public bool isFixed; //determina si el nodo se encuentra estatico

        public Vector3 offset; //variable que almacena la distancia entre la posicion del nodo y el centro del fixer con el que colisiona
        public Vector3 windforce; //variable que acumula la fuerza de todos los vientos de la escena que afecten a la tela
        public Vector3 penaltyforce; //variable que acumula la fuerza de penalty de los objetos que estan chocando contra el nodo

        public ElasticSolid manager;

        public Node(ElasticSolid m, Vector3 position, int id)
        {
            this.id = id;
            this.pos = position;
            this.vel = Vector3.zero;
            this.windforce = Vector3.zero;
            this.manager = m;
            this.isFixed = false;
        }
        public void SetParams(float damping)
        {
            this.damping = damping;
        }
        public void ComputeForces()
        {
            force += mass * manager.Gravity - damping * vel * mass;
            force += windforce;
            penaltyforce = Vector3.zero;
        }
    }
    public class Spring
    {
        public Node nodeA, nodeB;

        public float Length0; //distancia entre el nodoA y el nodoB en el instante inicial
        public float Length; //valor de la distancia entre ambos nodos en cada momento
        public float volume; //valor que almacena el volumen de los tetraedros de la lista de tetraedros para utilizarlo en el calculo de la fuerza

        public float stiffness; //coeficiente de rigidez del muelle. A mayor rigidez mas gomosa sera la tela
        public float damping;

        public List<Tetrahedron> aTetra; //paso la lista de tetraedros para calculart el volumen y usarlo en la formula
        public ElasticSolid manager;

        public Spring(ElasticSolid m, Node a, Node b, List<Tetrahedron> t)
        {
            this.manager = m;
            this.nodeA = a;
            this.nodeB = b;
            this.Length = (nodeA.pos - nodeB.pos).magnitude;
            this.Length0 = this.Length;
            this.aTetra = t;
            this.volume = 0.0f;

            foreach(Tetrahedron tet in t)
            {
                volume += tet.volume / 6;
            }
        }

        public void SetParams(float stiffness, float damping)
        {
            this.stiffness = stiffness;
            this.damping = damping;
        }

        public void ComputeForces()
        {
            Vector3 u = nodeA.pos - nodeB.pos;
            Length = u.magnitude;
            u.Normalize();

            volume = 0.0f;
            foreach (var tetra in aTetra)
            {
                volume += tetra.volume / 6;
            }

            float stress = -(volume / (Length * Length0)) * stiffness * (Length - Length0) + stiffness * damping * Vector3.Dot(u, nodeA.vel - nodeB.vel);
            Vector3 force = stress * u;
            //Vector3 force = -(volume / Length0 * Length0) * manager.density * (Length - Length0) * ((nodeA.pos - nodeB.pos) / Length); no funciona del todo bien
            nodeA.force += force;
            nodeB.force -= force;
        }
    }

    //clases de ayuda / auxiliares
    public class VertexInfo
    {
        //esta clase la utilizo para asignar los pesos a los vertices de la malla embebida dentro de la lista de tetraedros
        public int id;
        public int tetraId;

        public float weightA;
        public float weightB;
        public float weightC;
        public float weightD;

        public VertexInfo(int id, int tetraId, float a, float b, float c, float d)
        {
            this.id = id;
            this.tetraId = tetraId;
            this.weightA = a;
            this.weightB = b;
            this.weightC = c;
            this.weightD = d;
        }
    }
    public class Tetrahedron
    {
        public int id;
        public Node nodeA, nodeB, nodeC, nodeD; //nodos que componen el tetraedro

        public float volume;

        public Tetrahedron(int id, Node a, Node b, Node c, Node d, float density)
        {
            this.id = id;
            this.nodeA = a;
            this.nodeB = b;
            this.nodeC = c;
            this.nodeD = d;

            //formula sobre el calculo del volumen de un tetraedro
            volume = Mathf.Abs(Vector3.Dot(Vector3.Cross(nodeA.pos - nodeD.pos, nodeB.pos - nodeD.pos), nodeC.pos - nodeD.pos)) / 6;

            //asigno proporcionalmente a los nodos una masa segun su densidad
            this.nodeA.mass = density * volume / 4;
            this.nodeB.mass = density * volume / 4;
            this.nodeC.mass = density * volume / 4;
            this.nodeD.mass = density * volume / 4;
        }

        public void setParams(float d)
        {
            //de esta manera puedo actualizar la densidad en tiempo real y que la masa de los nodos se actualice
            this.nodeA.mass = d * volume / 4;
            this.nodeB.mass = d * volume / 4;
            this.nodeC.mass = d * volume / 4;
            this.nodeD.mass = d * volume / 4;
        }
    }
    public class Edge
    {
        //Clase Edge utilizada para simular y ordenar las aristas formadas por los nodos
        public int va, vb;
        public List<Tetrahedron> tetras = new List<Tetrahedron>();

        public Edge(int a, int b, Tetrahedron tetra)
        {
            if (a < b)
            {
                va = a;
                vb = b;
            }
            else
            {
                vb = a;
                va = b;
            }
            tetras.Add(tetra);
        }
    }
    public class EdgeComparer : IEqualityComparer<Edge>
    {
        //Clase EdgeComparer utilizada para ordenar segun el metodo Equals(Edge a, Edge b) que aristas son iguales
        //se situaran de forma consecutiva
        //metodo para ordenar y que el diccionario pueda comparar si los edges son iguales o no, en caso de ser iguales no se incluiran en el diccionario
        public bool Equals(Edge a, Edge b)
        {
            if (a.va == b.va && a.vb == b.vb || a.va == b.vb && a.vb == b.va) return true;
            else return false;
        }

        public int GetHashCode(Edge e)
        {
            List<int> idx = new List<int>();
            idx.Add(e.va);
            idx.Add(e.vb);
            idx.Sort();

            int hcode = ((idx[0] + idx[1]) * (idx[0] + idx[1] + 1)) / 2 + idx[1];

            return hcode.GetHashCode();
        }
    }

    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1,
    };

    public ElasticSolid()
    {
        //Constructor para definir los valores por defecto
        this.Paused = true;
        this.TimeStep = 0.02f;
        this.Substeps = 1;
        this.Gravity = new Vector3(0.0f, -9.81f, 0.0f);
        this.IntegrationMethod = Integration.Symplectic;
    }

    #region InEditorVariables
    public TextAsset nodesFile;
    public TextAsset tetrasFile;

    public Integration IntegrationMethod;
    public bool Paused;
    public float TimeStep; private float newTimeStep;
    public int Substeps;
    public Vector3 Gravity;

    public float density;
    public float nodeDamping;
    public float springStiffnessTraccion;
    public float springDamping;
    public float windFrictionCoeficient;
    public float penaltyCoeficient;

    public List<Node> nodes;
    public List<Spring> springs;
    public List<Tetrahedron> tetras;
    public List<Edge> edges;
    public List<VertexInfo> vertexInfos;

    public List<Fixer> fixers; //lista de fixer que afectan a la tela
    public List<Wind> winds; //lista de vientos que afectan a la tela

    public List<GameObject> spheres; //lista de las esferas con los que la tela colisiona
    public List<GameObject> cubes; //lista de los cubos con los que la tela colisiona
    #endregion

    public void Start()
    {
        newTimeStep = TimeStep / Substeps;

        nodes = new List<Node>();
        springs = new List<Spring>();
        tetras = new List<Tetrahedron>();
        edges = new List<Edge>();
        vertexInfos = new List<VertexInfo>();

        createNodes(nodesFile, tetrasFile);
        vertexInTetra();
        createSprings();
        fixNodes();
    }

    public void Update()
    {
        //Al pulsar "P" las fisicas de la tela se detendran
        if (Input.GetKeyUp(KeyCode.P))
            this.Paused = !this.Paused;

        newTimeStep = TimeStep / Substeps;

        Mesh mesh = this.GetComponent<MeshFilter>().mesh;  //malla de la tela
        Vector3[] vertices = new Vector3[mesh.vertexCount]; //array para almacenar las nuevas posiciones de los nodos calculadas en la fisica

        //Para cada nodo que se encuentra fixeado le actualizo la posicion a la de su fixer
        //De esta manera la mover o rotar el fixer los nodos se moveran junto a el
        foreach (Node node in nodes)
        {
            if (node.isFixed)
            {
                node.pos = node.fixer.transform.TransformPoint(node.offset);
            }
        }

        for (int i = 0; i < mesh.vertexCount; i++)
        {
            vertices[i] = transform.InverseTransformPoint(vertexInfos[i].weightA * tetras[vertexInfos[i].tetraId].nodeA.pos +
                                                          vertexInfos[i].weightB * tetras[vertexInfos[i].tetraId].nodeB.pos +
                                                          vertexInfos[i].weightC * tetras[vertexInfos[i].tetraId].nodeC.pos +
                                                          vertexInfos[i].weightD * tetras[vertexInfos[i].tetraId].nodeD.pos);
        }
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }

    public void FixedUpdate()
    {
        //Si esta pausado no continuo ejecutando
        if (this.Paused)
            return;

        //Dependiendo del metodo de integracion seleccionado se llamara a una u otra funcion de calculo de fisicas
        //La funcion se ejecuta por cada llamada al FixedUpdate Substeps veces.
        for (int i = 0; i < Substeps; i++)
        {
            switch (this.IntegrationMethod)
            {
                case Integration.Explicit: this.stepExplicit(); break;
                case Integration.Symplectic: this.stepSymplectic(); break;
                default: throw new System.Exception("[ERROR] Should never happen!");
            }
        }
    }

    public void OnDrawGizmos()
    {
        foreach (Spring spring in springs)
        {
            printGizmos(spring.nodeA.pos, spring.nodeB.pos, Color.cyan);
        }
    }
    private void stepExplicit()
    {
        //Para cada nodo establezco la masa y el damping en tiempo real y calculo la fuerza correspondiente
        foreach (Node node in nodes)
        {
            node.SetParams(nodeDamping);
            node.force = Vector3.zero;
            node.ComputeForces();
        }

        //Para cada tipo de muelle establezco la rigidez y el damping y calculo la fuerza correspondiente
        foreach (Spring spring in springs)
        {
            spring.SetParams(springStiffnessTraccion, springDamping);
            spring.ComputeForces();
        }

        //Para cada nodo que no esta fixeado calculo el movimiento
        foreach (Node node in nodes)
        {
            if (!node.isFixed)
            {
                foreach (GameObject sphere in spheres)
                {
                    if (isPointInsideSphere(node.pos, sphere))
                    {
                        //sphere colision
                        Vector3 point = computeSphereColisions(sphere, node.pos);
                        computePenaltyForce(penaltyCoeficient, point, node);
                    }
                }
                foreach (GameObject cube in cubes)
                {
                    if (isPointInsideCube(node.pos, cube))
                    {
                        //cube colision
                        Vector3 point = computeCubeColisions(cube);
                        computePenaltyForce(penaltyCoeficient, point, node);
                    }
                }
                node.pos += newTimeStep * node.vel;
                node.vel += newTimeStep / node.mass * node.force;
            }
        }
    }
    private void stepSymplectic()
    {
        foreach (Tetrahedron tetra in tetras)
        {
            tetra.setParams(density);
        }

        //Para cada nodo establezco la masa y el damping en tiempo real y calculo la fuerza correspondiente
        foreach (Node node in nodes)
        {
            node.SetParams(nodeDamping);
            node.force = Vector3.zero;
            node.ComputeForces();
        }

        //Para cada tipo de muelle establezco la rigidez y el damping y calculo la fuerza correspondiente
        foreach (Spring spring in springs)
        {
            spring.SetParams(springStiffnessTraccion, springDamping);
            spring.ComputeForces();
        }


        //Para cada nodo que no esta fixeado calculo el movimiento
        foreach (Node node in nodes)
        {
            if (!node.isFixed)
            {
                foreach(GameObject sphere in spheres)
                {
                    if (isPointInsideSphere(node.pos, sphere))
                    {
                        //sphere colision
                        Vector3 point = computeSphereColisions(sphere, node.pos);
                        computePenaltyForce(penaltyCoeficient, point, node);
                    }
                }
                foreach (GameObject cube in cubes)
                {
                    if (isPointInsideCube(node.pos, cube))
                    {
                        //cube colision
                        Vector3 point = computeCubeColisions(cube);
                        computePenaltyForce(penaltyCoeficient, point, node);
                    }
                }
                node.vel += newTimeStep / node.mass * node.force;
                node.pos += newTimeStep * node.vel;
            }
        }
    }

    #region FUNCIONES PARA LA GENERACION DEL SISTEMA MASA MUELLE Y SU VISUALIZACION CON GIZMOS

    void createNodes(TextAsset nodesFile, TextAsset tetrasFile)
    {
        CultureInfo locale = new CultureInfo("en-US");

        string[] nodesRaw = nodesFile.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);
        string[] tetraRaw = tetrasFile.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);

        //creo los nodos en sus posiciones correspondientes al fichero
        for(int i = 4; i < nodesRaw.Length; i += 4)
        {
            Vector3 pos = transform.TransformPoint(new Vector3(float.Parse(nodesRaw[i + 1], locale), float.Parse(nodesRaw[i + 2], locale), float.Parse(nodesRaw[i + 3], locale)));
            nodes.Add(new Node(this, pos, int.Parse(nodesRaw[i + 0], locale)));
        }

        //creo los tetraedros
        for(int i = 3; i < tetraRaw.Length; i += 5)
        {
            //el indice en el archivo .ele empieza por 1 y el primer indice de una lista es 0 por tanto le resto 1 a los valores. Si no hiciese esto tendria un index outboundexception
            tetras.Add(new Tetrahedron(
                int.Parse(tetraRaw[i + 0], locale) - 1,
                nodes[int.Parse(tetraRaw[i + 1], locale) - 1],
                nodes[int.Parse(tetraRaw[i + 2], locale) - 1],
                nodes[int.Parse(tetraRaw[i + 3], locale) - 1],
                nodes[int.Parse(tetraRaw[i + 4], locale) - 1],
                density));
        }
    }
    void createSprings()
    {
        //creo los edges, los almaceno en un diccionario para eliminar duplicados y una vez eliminados, creo los springs correspondientes
        EdgeComparer edgeComparer = new EdgeComparer();
        Dictionary<Edge, Edge> eDictionary = new Dictionary<Edge, Edge>(edgeComparer);

        Edge e;
        for(int i = 0; i < tetras.Count; i++)
        {
            //edeges que formarian un triangulo de nodos a b c
            edges.Add(new Edge(tetras[i].nodeA.id, tetras[i].nodeB.id, tetras[i]));
            edges.Add(new Edge(tetras[i].nodeB.id, tetras[i].nodeC.id, tetras[i]));
            edges.Add(new Edge(tetras[i].nodeC.id, tetras[i].nodeA.id, tetras[i]));

            //edges restantes hasta el nodo d para formar tetraedro
            edges.Add(new Edge(tetras[i].nodeD.id, tetras[i].nodeA.id, tetras[i]));
            edges.Add(new Edge(tetras[i].nodeD.id, tetras[i].nodeB.id, tetras[i]));
            edges.Add(new Edge(tetras[i].nodeD.id, tetras[i].nodeC.id, tetras[i]));

            foreach (var edge in edges)
            {
                if (!eDictionary.TryGetValue(edge, out e)) eDictionary.Add(edge, edge);
                else e.tetras.Add(tetras[i]);
            }
        }

        //una vez tengo los edges sin repeticiones creo los springs
        foreach(var edge in eDictionary)
        {
            springs.Add(new Spring(this, nodes[edge.Value.va-1], nodes[edge.Value.vb-1], edge.Value.tetras));
        }

    }
    void fixNodes()
    {
        //Fixea cada nodo que se encuentra en contacto con un fixer de la lista fixers
        foreach (Node node in nodes)
        {
            foreach (Fixer fixer in fixers)
            {
                if (fixer.CalculateCollision(node.pos))
                {
                    node.isFixed = true;
                    node.fixer = fixer;
                    node.offset = fixer.transform.InverseTransformPoint(node.pos); //almaceno la distancia entre el nodo y el centro del fixer
                }
            }
        }
    }
    void vertexInTetra()
    {
        //compruebo que vertices de la malla se encuentran dentro de que tetraedros para de esta manera asignarles los pesos correspondientes
        Mesh mesh = this.GetComponent<MeshFilter>().mesh;

        for (int i = 0; i < mesh.vertexCount; i++)
        {
            Vector3 pos = transform.TransformPoint(mesh.vertices[i]);
            foreach (var tetra in tetras)
            {
                if (isPointInsideTetraedron(pos, tetra))
                {
                    //calculo los pesos correspondientes a cada vertice respecto al tetraedro en el que se encuentra con las normales de las caras para afuera
                    Vector3 diffA = tetra.nodeA.pos - pos;
                    Vector3 diffB = tetra.nodeB.pos - pos;
                    Vector3 diffC = tetra.nodeC.pos - pos;
                    Vector3 diffD = tetra.nodeD.pos - pos;

                    //dados los 4 nodos de un tetraedro el volumen del tetraedro se puede calcular como:
                    float vA = (Mathf.Abs(Vector3.Dot(Vector3.Cross(diffB, diffC), diffD)) / 6);
                    float vB = (Mathf.Abs(Vector3.Dot(Vector3.Cross(diffA, diffC), diffD)) / 6);
                    float vC = (Mathf.Abs(Vector3.Dot(Vector3.Cross(diffA, diffB), diffD)) / 6);
                    float vD = (Mathf.Abs(Vector3.Dot(Vector3.Cross(diffA, diffB), diffC)) / 6);

                    // formula de los pesos Wi = Vi / V
                    float weightA = vA / tetra.volume;
                    float weightB = vB / tetra.volume;
                    float weightC = vC / tetra.volume;
                    float weightD = vD / tetra.volume;

                    vertexInfos.Add(new VertexInfo(i, tetra.id, weightA, weightB, weightC, weightD));
                    break;
                }

            }
        }
    }
    public void printGizmos(Vector3 pos1, Vector3 pos2, Color color)
    {
        //Pinto los nodos
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(pos1, 0.2f);
        Gizmos.DrawSphere(pos2, 0.2f);

        //Pinto las aristas
        Gizmos.color = color;
        Gizmos.DrawLine(pos1, pos2);
    }

    #endregion

    #region FUNCIONES PARA EL CALCULO DE COLISIONES CON OTROS GAMEOBJECTS

    public Vector3 computeCubeColisions(GameObject cube)
    {
        // se detiene al impactar con la superficie superior del cubo
        BoxCollider collider = cube.GetComponent<BoxCollider>();
        return collider.transform.up;
    }
    public Vector3 computeSphereColisions(GameObject sphere, Vector3 pos)
    {

        SphereCollider collider = sphere.GetComponent<SphereCollider>();
        Vector3 point = pos + Vector3.ClampMagnitude(pos - collider.transform.position, collider.radius * sphere.transform.lossyScale.x);
        return point;
    }
    public Vector3 computePlaneColisions(GameObject plane, Vector3 pos)
    {
        //no implementado aun
        return Vector3.zero;
    }
    public void computePenaltyForce(float penaltyCoeficient, Vector3 point, Node node)
    {
        //formula de la fuerza de penalty
        Vector3 normal = node.pos - point;
        normal.Normalize();

        node.penaltyforce += -penaltyCoeficient * normal.magnitude * normal;
        node.force += node.penaltyforce;
    }
    public bool isPointInsideSphere(Vector3 position, GameObject sphere)
    {
        //Si la distancia entre el nodo y el centro de la esfera es menor que el radio eso significa que se encuentra dentro y por tanto hay contacto
        var distance = Math.Sqrt((position.x - sphere.transform.position.x) * (position.x - sphere.transform.position.x) +
                                 (position.y - sphere.transform.position.y) * (position.y - sphere.transform.position.y) +
                                 (position.z - sphere.transform.position.z) * (position.z - sphere.transform.position.z));
        float radius = sphere.GetComponent<SphereCollider>().radius * sphere.transform.localScale.x;
        return distance <= radius;
    }
    public bool isPointInsideCube(Vector3 position, GameObject cube)
    {
        //Si la posicion dada se encuentra dentro de los limites del cubo con la formula de AABB
        var minX = cube.transform.position.x - 0.5 * cube.transform.localScale.x;
        var maxX = cube.transform.position.x + 0.5 * cube.transform.localScale.x;

        var minY = cube.transform.position.y - 0.5 * cube.transform.localScale.y;
        var maxY = cube.transform.position.y + 0.5 * cube.transform.localScale.y;

        var minZ = cube.transform.position.z - 0.5 * cube.transform.localScale.z;
        var maxZ = cube.transform.position.z + 0.5 * cube.transform.localScale.z;

        return (position.x >= minX && position.x <= maxX) &&
               (position.y >= minY && position.y <= maxY) &&
               (position.z >= minZ && position.z <= maxZ);
    }
    public bool isPointInsideTetraedron(Vector3 position, Tetrahedron tetra)
    {
        Vector3 n1 = Vector3.Cross(tetra.nodeB.pos - tetra.nodeA.pos, tetra.nodeC.pos - tetra.nodeA.pos);
        Vector3 n2 = Vector3.Cross(tetra.nodeC.pos - tetra.nodeA.pos, tetra.nodeD.pos - tetra.nodeA.pos);
        Vector3 n3 = Vector3.Cross(tetra.nodeD.pos - tetra.nodeA.pos, tetra.nodeB.pos - tetra.nodeA.pos);
        Vector3 n4 = Vector3.Cross(tetra.nodeD.pos - tetra.nodeB.pos, tetra.nodeC.pos - tetra.nodeB.pos);

        return n1.x * (tetra.nodeA.pos.x - position.x) + n1.y * (tetra.nodeA.pos.y - position.y) + n1.z * (tetra.nodeA.pos.z - position.z) <= 0.0001 &&
               n2.x * (tetra.nodeA.pos.x - position.x) + n2.y * (tetra.nodeA.pos.y - position.y) + n2.z * (tetra.nodeA.pos.z - position.z) <= 0.0001 &&
               n3.x * (tetra.nodeA.pos.x - position.x) + n3.y * (tetra.nodeA.pos.y - position.y) + n3.z * (tetra.nodeA.pos.z - position.z) <= 0.0001 &&
               n4.x * (tetra.nodeD.pos.x - position.x) + n4.y * (tetra.nodeD.pos.y - position.y) + n4.z * (tetra.nodeD.pos.z - position.z) <= 0.0001;

    }

    #endregion

}
