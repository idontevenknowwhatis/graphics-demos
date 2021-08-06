// TRIANGLE MESH
class MESH {
    // VERTICES
    int nv=0, maxnv = 1000;  
    pt[] G = new pt [maxnv];                        
    // TRIANGLES 
    int nt = 0, maxnt = maxnv*2;                           
    boolean[] isInterior = new boolean[maxnv];                                      
    // CORNERS 
    int c=0;    // current corner                                                              
    int nc = 0; 
    int[] V = new int [3*maxnt];   
    int[] O = new int [3*maxnt];
    int[] borderPoints;
    int borders = 0;
    
    // current corner that can be edited with keys
  MESH() {for (int i=0; i<maxnv; i++) G[i]=new pt();};
  void reset() {nv=0; nt=0; nc=0;}                                                  // removes all vertices and triangles
  void loadVertices(pt[] P, int n) {nv=0; for (int i=0; i<n; i++) addVertex(P[i]);}
  void writeVerticesTo(pts P) {for (int i=0; i<nv; i++) P.G[i].setTo(G[i]);}
  void addVertex(pt P) { G[nv++].setTo(P); }                                             // adds a vertex to vertex table G
  void addTriangle(int i, int j, int k) {

V[nc++]=i; V[nc++]=j; V[nc++]=k; nt=nc/3; }     // adds triangle (i,j,k) to V table

  // CORNER OPERATORS
  int t (int c) {int r=int(c/3); return(r);}                   // triangle of corner c
  int n (int c) {int r=3*int(c/3)+(c+1)%3; return(r);}         // next corner
  int p (int c) {int r=3*int(c/3)+(c+2)%3; return(r);}         // previous corner
  pt g (int c) {return G[V[c]];}                             // shortcut to get the point where the vertex v(c) of corner c is located

  boolean nb(int c) {return(O[c]!=c);};  // not a border corner
  boolean bord(int c) {return(O[c]==c);};  // not a border corner

  pt cg(int c) {return P(0.6,g(c),0.2,g(p(c)),0.2,g(n(c)));}   // computes offset location of point at corner c

  // CORNER ACTIONS CURRENT CORNER c
  void next() {c=n(c);}
  void previous() {c=p(c);}
  void opposite() {c=o(c);}
  void left() {c=l(c);}
  void right() {c=r(c);}
  void swing() {c=s(c);} 
  void unswing() {c=u(c);} 
  void printCorner() {println("c = "+c);}
  
  

  // DISPLAY
  void showCurrentCorner(float r) { if(bord(c)) fill(red); else fill(dgreen); show(cg(c),r); };   // renders corner c as small ball
  void showEdge(int c) {beam( g(p(c)),g(n(c)),rt ); };  // draws edge of t(c) opposite to corner c
  void showVertices(float r) // shows all vertices green inside, red outside
    {
    for (int v=0; v<nv; v++) 
      {
      if(isInterior[v]) fill(green); else fill(red);
      show(G[v],r);
      }
    }                          
  void showInteriorVertices(float r) {for (int v=0; v<nv; v++) if(isInterior[v]) show(G[v],r); }                          // shows all vertices as dots
  void showTriangles() { for (int c=0; c<nc; c+=3) show(g(c), g(c+1), g(c+2)); }         // draws all triangles (edges, or filled)
  void showEdges() {for (int i=0; i<nc; i++) showEdge(i); };         // draws all edges of mesh twice
  
  boolean flatterThan(pt A, pt B, pt C, float r){
    return triThickness(A,B,C)< r;
  }
  
  void triangulate()      // performs Delaunay triangulation using a quartic algorithm
   {
   c=0;                   // to reset current corner
   // **01 implement it
  //for (int i=0; i<nv; i++) for (int j=0; j<nv; j++) for (int k=0; k<nv; k++) {
    boolean thinTriangleTest = false;
    for (int i=0; i<nv-2; i++) for (int j=i+1; j<nv-1; j++) for (int k=j+1; k<nv; k++) {
        if(i != j && i != k && j !=k){
            pt v1 = G[i];
            pt v2 = G[j];
            pt v3 = G[k];
            pt center = CircumCenter(v1,v2,v3);
            float radius = d(center,G[i]);
            
            boolean sparse = true;
            for(int t = 0; t < G.length; t++){
              pt test = G[t];
              if(d(test,center) < (radius - 0.01)){
                sparse = false;
                break;
              }
            }
            if(sparse){
              int index1;
              int index2;
              int index3;
              if(!ccw(v1,v2,v3)){
                index1 = i;
                index2 = k;
                index3 = j;
              } else {
                index1 = i;
                index2 = j;
                index3 = k;
              }
              if(useColinearTest){
                if(flatterThan(v1,v2,v3,10)){
                  print("thin triangle found\n");
                  thinTriangleTest = true;
                  continue;
                } else {
                  addTriangle(index1,index2,index3);
                }
              } else {
                addTriangle(index1,index2,index3);
              }
            }
        } else {
          continue;

        }
  }
  thinTriangleWarning = thinTriangleTest;
  
   } 
  
  boolean opposites(int cornerIndex, int candidateIndex){
    return (V[n(cornerIndex)] == V[p(candidateIndex)]) && (V[p(cornerIndex)] == V[n(candidateIndex)]);
  }

   
  void computeO() // **02 implement it 
    {                                          
    // **02 implement it
    for(int index = 0; index < nc; index++){
      O[index] = index;
    }
    for(int cornerIndex = 0; cornerIndex < nc; cornerIndex++){
      for(int candidateIndex = 0; candidateIndex < nc; candidateIndex++){
          if(opposites(cornerIndex,candidateIndex)){
            O[cornerIndex] = candidateIndex;
          }
        
      }
    }
    }
    
  int computeCorner(int vertex){
    for(int i = 0; i < nc; i++){
      if(V[i] == vertex){
         return i;
      }
    }
    return -1;
  }
  int computeCornersCount(int vertex){
    int size = 1;
    int startCorner = computeCorner(vertex);
    int currentCorner = computeCorner(vertex);
    while(s(currentCorner) != startCorner){
      size++;
      currentCorner = s(currentCorner);
    }
    return size;
  }
  int[] computeCorners(int vertex){
    int size = computeCornersCount(vertex);
    int[] corners = new int[size];
    int startCorner = computeCorner(vertex);
    int currentCorner = computeCorner(vertex);
    int loc = 0;
    corners[loc] = currentCorner;
    while(s(currentCorner)!= startCorner){
      loc++;
      currentCorner = s(currentCorner);
      corners[loc] = currentCorner;
    }
    return corners;
  }
  int[] computeVertices(int[] corners){
    int size = corners.length;
    int[] vertices = new int[size];
    for(int i = 0; i < size; i++){
      vertices[i] = V[n(corners[i])];
    }
    return vertices;
  }
  
  pt computeTargetVertex(int[] vertices){
    float scaling = vertices.length;
    float avgX = 0;
    float avgY = 0;
    float avgZ = 0;
    for(int i = 0; i < vertices.length; i++){
      avgX += G[vertices[i]].x;
      avgY += G[vertices[i]].y;
      avgZ += G[vertices[i]].z;
    }
    avgX = avgX/scaling;
    avgY = avgY/scaling;
    avgZ = avgZ/scaling;
    pt out = new pt();
    out.x = avgX;
    out.y = avgY;
    out.z = avgZ;
    return out;
  }
  void showBorderEdges()  // draws all border edges of mesh
    {
    // **02 implement; 
    int borders1 = 0;
    
    for(int i = 0; i < nc; i++){
      if(O[i] == i){
        caplet(G[V[p(i)]],20,G[V[n(i)]],20);
        borders1++;
      }
    }
    borders = borders1;
    }

  void showNonBorderEdges() // draws all non-border edges of mesh
    {
    // **02 implement 
    for(int i = 0; i < nc; i++){
      if(O[i] != i){
        caplet(G[V[p(i)]],20,G[V[n(i)]],20);
      }
    }
    }        
    
  void classifyVertices() 
    { 
    // **03 implement it 
    for(int i = 0; i < nc; i++){
      isInterior[V[i]] = true;
    }
    for(int i = 0; i < nc; i++){
      if(O[i] == i){
        //index = V[i]
        isInterior[V[p(i)]] = false;
        isInterior[V[n(i)]] = false;
      }
    }
    }  
    
  void smoothenInterior() { // even interior vertiex locations
    pt[] Gn = new pt[nv];
    // **04 implement it 
    for (int i = 0; i < nv; i++){
      if(isInterior[i]){
        int[] corners = computeCorners(i);
        int[] vertices = computeVertices(corners);
        pt target = computeTargetVertex(vertices);
        Gn[i] = target;
      }
    }
    for (int v=0; v<nv; v++) if(isInterior[v]) G[v].translateTowards(.1,Gn[v]);
    }


   // **05 implement corner operators in Mesh
  int v (int c) {return V[c];}                                // vertex of c
  int o (int c) {return O[c];}                                // opposite corner
  int l (int c) {
    int next = n(c);
    if(o(next) == next){
      return c;
    } else {
      return o(next);
    }
  }                             // left
  int s (int c) {
    if(l(c) == c){
      return c;
    } else {
      return n(l(c));
    }
}
  int r (int c){// {return O[p(c)];} 
    int prev = p(c);
    if(o(prev) == prev){
        return c;
    } else {
      return o(prev);
    }
  }
        
  int u (int c) {
    //return p(r(c));
    if(r(c) == c){
      return c;
    } else {
      return p(r(c));
    }
}                            
        //int n = n(i);
        //int p = p(i);
        //pt mid = P(G[V[i]],G[V[O[i]]],G[V[n]],G[V[p]]);
        ////fill(blue);sphere(mid,30);
        //drawParabola(G[V[i]],mid,G[
        //num++;
  void showOpposites(){
    //int num = 0;
    for (int i=0; i<nc-2; i++) for (int j=i+1; j<nc-1; j++){
      if(o(i) == i || o(j) == j){
        continue;
      } else if(o(i) != j || o(j) != i){
        continue;
      } else {
        int n = n(i);
        int p = p(i);
        pt mid;
        if(useCenterOfMassMidpoint){
          mid = P(G[V[i]],G[V[O[i]]],G[V[n]],G[V[p]]);
        } else {
          mid = P(G[V[n]],G[V[p]]);
        }
        //fill(blue);sphere(mid,30);
        beginShape();
        fill(blue);drawParabolaInHat(G[V[i]],mid,G[V[j]],9);
        endShape();
      }
      
    }
  }
  pt[] getTriangleVertices(int corner){
    pt t1 = G[V[corner]];
    pt t2 = G[V[n(corner)]];
    pt t3 = G[V[p(corner)]];
    pt[] vertices = new pt[3];
    vertices[0] = t1;
    vertices[1] = t2;
    vertices[2] = t3;
    return vertices;
  }

  void showVoronoiEdges() // draws Voronoi edges on the boundary of Voroni cells of interior vertices
    { 
    for (int i = 0; i < nv; i++){
      if(isInterior[i]){
        int[] corners = computeCorners(i);
        pt[] circumCenters = new pt[corners.length];
        for(int j = 0; j < corners.length; j++){
          circumCenters[j] = triCircumcenter(corners[j]);
        }
        for(int k = 0; k < (corners.length - 1); k++){
          //fill(blue);sphere(circumCenters[k],30);
          show(circumCenters[k], circumCenters[k+1]);
        }
        show(circumCenters[circumCenters.length - 1], circumCenters[0]);
      }
    }
    
    }
                   

  void showArcs() // draws arcs of quadratic B-spline of Voronoi boundary loops of interior vertices
    { 
    
      for (int i = 0; i < nv; i++){
      if(isInterior[i]){
        int[] corners = computeCorners(i);
        pt[] circumCenters = new pt[corners.length];
        for(int j = 0; j < corners.length; j++){
          circumCenters[j] = triCircumcenter(corners[j]);
        }
        
        for(int k = 0; k < circumCenters.length; k++){
          pt b = circumCenters[k];
          int prev = (k-1) % circumCenters.length;
          if(prev < 0){
            prev = prev + circumCenters.length;
          }
          int next = (k+1) % circumCenters.length;
          
          pt prevMid = P(circumCenters[prev], b);
          pt nextMid = P(circumCenters[next], b);
          drawParabolaInHat(prevMid,b,nextMid,4);
        }
       
      }
    }
    }               // draws arcs in triangles

void drawVoronoiCell(int vertex){
  if(isInterior[vertex]){
    int[] corners = computeCorners(vertex);
    pt[] circumCenters = new pt[corners.length];
    for(int j = 0; j < corners.length; j++){
      circumCenters[j] = triCircumcenter(corners[j]);
    }
    beginShape();
    for(int k = 0; k < circumCenters.length; k++){
      vertex(circumCenters[k]);
    }
    endShape();
  }
}

void drawVoronoiCells(){
  float dc = 1./(nv-1);
  for(int v = 0; v < nv; v++){
    if(isInterior[v]){
      fill(dc * 255 * v, dc * 255 * (nv -v), 200);
      drawVoronoiCell(v);
    }
  }
}
    
 
  pt triCenter(int c) {return P(g(c),g(n(c)),g(p(c))); }  // returns center of mass of triangle of corner c
  pt triCircumcenter(int c) {return CircumCenter(g(c),g(n(c)),g(p(c))); }  // returns circumcenter of triangle of corner c


  } // end of MESH
