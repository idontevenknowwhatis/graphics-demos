class robot{
  MESH m;
  pts R;
  pt mouse;
  
  //positional info
  int originCorner;
  int destinationCorner;
  pt currentLocation;
  float speed;
  int collisions;
  int movementState = 0;
  pt previousLocation;
  float robotHeight;
  float volume;
  int rays = 30;
  float reducedPathDistance = 0;
  float userPathDistance = 0;
  
  
  //Point bookkeepiing
  HashMap<Integer, Boolean> visited = new HashMap<Integer, Boolean>();
  HashMap<Integer, Boolean> detectedPillars = new HashMap<Integer, Boolean>();
  
  //adjacency info
  int[][] adjacentCornerTable;
  
  //path bookkeeping
  int usersteps = 0;
  int userlength = -1;
  int[] searchpath;
  int pathcounter = 1;
  int reducedPathLength;
  
  robot(MESH m1, pts r1, int corner){
    this.m = m1;
    this.R = r1;
    originCorner = corner;
    M.reset();
    M.loadVertices(R.G,R.nv);
    M.triangulate();
    //M.showTriangles();
    M.computeO();
    M.classifyVertices();
    destinationCorner = M.o(corner);
    currentLocation = M.triCircumcenter(corner);
    adjacentCornerTable = new int[M.nc][];
    //print(currentLocation.x);
    speed = 2;
    for(int i = 0; i < M.nc; i++){
      visited.put(i, false);
    }
    visited.put(corner, true);
    visitCorners(originCorner);
    setAdjacencyTable();
    int[][] red = computeReducedCornerTable();
    dfs(red, corner);
    
    println(min(new int[] {-1,4}));
    for(int i = 0; i < M.nv; i++){
      detectedPillars.put(i,false);
    }
    reducedPathLength = getReducedPathLength();
    println("search path computed of steps = "+ searchpath.length, " distance = ", reducedPathDistance);
    volume = 200 * 300 * 50;
    
  }
  void draw(){
    pushMatrix(); 
    translate(0,0,4); noFill(); stroke(yellow);
    M.reset();
    M.loadVertices(R.G,R.nv);
    M.triangulate();
    M.showTriangles();
    popMatrix();
    noStroke();
    
    M.computeO();
    M.classifyVertices();
    updateSpeed();
    float rad = computeNearestCornerDistance();
    renderRobot(rad);
    moveRobot();
    previousLocation = P(currentLocation,new vec(0,0,0));
    showCurrentDestination();
    showCurrentOrigin();
    int currentCorner = computeCurrentCorner();
    fill(orange);sphere(M.triCircumcenter(currentCorner),30);
    int[][] red = computeReducedCornerTable();
    for(int i = 0; i < red[destinationCorner/3].length; i++){
      fill(white);sphere(M.triCircumcenter(red[destinationCorner/3][i] * 3),20);
    }
    touchPillars(rad);
    
    if(detectingPillars == false){
      drawVisited();
      if(followingPath){
        if(atPoint(M.triCircumcenter(destinationCorner))){
          userPathDistance += d(M.triCircumcenter(originCorner), M.triCircumcenter(destinationCorner));
          pathcounter++;
          visitCorners(destinationCorner);
          if(!visited.containsValue(false)){
            followingPath = false;
            println("robot finished cleaning, total steps = " + pathcounter, "distance travelled = " + userPathDistance);
            doPick();
          }
          destinationCorner = computeNearestCircumcenterCorner();
          originCorner = M.o(destinationCorner);
          
        }
        fill(blue);sphere(M.triCircumcenter(searchpath[pathcounter]*3),50);
      } else {
        drawMouse();
        if(atPoint(M.triCircumcenter(destinationCorner))){
          userPathDistance += d(M.triCircumcenter(originCorner), M.triCircumcenter(destinationCorner));
          usersteps++;
          visitCorners(destinationCorner);
          if(!visited.containsValue(false)){
            println("User finished cleaning, total steps = " + usersteps, "distance travelled = " + userPathDistance);
             if(usersteps < reducedPathLength){
              println("THE USER BEAT THE ROBOT");
            } else {
              println("THE ROBOT BEAT THE USER");
            }
            userlength = usersteps;
          }
          destinationCorner = computeNearestCircumcenterCorner();
          originCorner = M.o(destinationCorner);
        }
      }
    }
    if(detectingPillars == true){
      int[] hit = shootRays(rays);
      for(int i = 0; i < hit.length; i++){
        detectedPillars.put(hit[i],true);
      }
      showSeenPillars();
      if(followingPath){
        if(atPoint(M.triCircumcenter(destinationCorner))){
          userPathDistance += d(M.triCircumcenter(originCorner), M.triCircumcenter(destinationCorner));
          pathcounter++;
          if(!detectedPillars.containsValue(false) && usersteps == 0){
            followingPath = false;
            println("all pillars spotted, total steps = " + pathcounter, "distance travelled = " + userPathDistance);
            doPick();
          }
          destinationCorner = computeNearestCircumcenterCorner();
          originCorner = M.o(destinationCorner);
          
        }
        fill(blue);sphere(M.triCircumcenter(searchpath[pathcounter]*3),50);
      } else {
        drawMouse();
        if(atPoint(M.triCircumcenter(destinationCorner))){
          userPathDistance += d(M.triCircumcenter(originCorner), M.triCircumcenter(destinationCorner));
          usersteps++;
          if(!detectedPillars.containsValue(false) && userlength == -1){
            println("User finished detecting all pillars, total steps = " + usersteps, "distance travelled = " + userPathDistance);
            doPick();
            userlength = usersteps;
          }
          destinationCorner = computeNearestCircumcenterCorner();
          originCorner = M.o(destinationCorner);
        }
      }
    } 
    
  }
    
  
  
  void updateSpeed(){
    if(robotPause){
      speed = 0;
      return;
    } else {
      
    if(movementState == 0){
      speed = 2;
    } else if(movementState == 1){
      float dist = sqrt(d(currentLocation, Of))/20;
      
      if(dist > 3){
        speed = 3;
      } else {
        speed = 0.5 + dist;
      }
    } else if(movementState == 2 || followingPath){
      float const1 = d(currentLocation, M.triCircumcenter(destinationCorner));
      float const2 = d(currentLocation, M.triCircumcenter(originCorner));
      float const3;
      if(const2 < const1){
        const3 = const2;
      } else {
        const3 = const1;
      }
      speed = 0.5 + const3/30;
    } 
  }
  }
  int[][] computeReducedCornerTable(){
    int[][] arr = new int[M.nc/3][];
    for(int i = 0; i < M.nc; i+= 3){
      int[] temp = new int[adjacentCornerTable[i].length];
      for(int l = 0; l < adjacentCornerTable[i].length; l++){
        temp[l] = adjacentCornerTable[i][l]/3;
      }
      arr[i/3] = temp;
    }
    return arr;
  }
  void visitCorners(Integer corner){
    visited.put(corner, true);
    visited.put(M.p(corner), true);
    visited.put(M.n(corner),true);
  }

  
  int[] computeAdjacentCorner(int i){
      int c1 = i;
      int c2 = M.n(i);
      int c3 = M.p(i);
      
      int[] corners = new int[3];
      corners[0] = c1;
      corners[1] = c2;
      corners[2] = c3;
      int numAdjacent = 0;
      for(int k = 0; k < 3; k++){
        if(M.o(corners[k]) != corners[k]){
          numAdjacent++;
        }
      }
      
      int[] adjacent = new int[numAdjacent];
      int loc = 0;
      for(int l = 0; l < 3; l++){
        if(M.o(corners[l]) != corners[l]){
          adjacent[loc] = M.o(corners[l]);
          loc++;
        }
      }
      return adjacent;
  }
  void setAdjacencyTable(){
    for(int i = 0; i < adjacentCornerTable.length; i++){
      adjacentCornerTable[i] = computeAdjacentCorner(i);
    }
  }
  
  int getPathLength(){
    return searchpath.length;
  }
  int getReducedPathLength(){
    HashMap<Integer, Boolean> tempvisited = new HashMap<Integer, Boolean>();
    for(int j = 0; j < M.nc/3; j++){
      tempvisited.put(j,false);
    }
    int steps = 0;
    //println(tempvisited.size());
    for(int i = 0; i < searchpath.length; i++){
      tempvisited.put(searchpath[i], true);
      steps++;
      reducedPathDistance += d(M.triCircumcenter(searchpath[i]), M.triCircumcenter(searchpath[i+1]));
      if(!tempvisited.containsValue(false)){
        break;
      }
    }
    return steps;
  }
      
  void dfs(int[][] reducedCornerTable, int startingcorner){
    boolean[] visited = new boolean[reducedCornerTable.length];
    for(int i = 0; i < visited.length; i++){visited[i] = false;}
    ArrayList<Integer[]> paths = new ArrayList<Integer[]>();
    visited[startingcorner/3] = true;
    println("READY");
    recdfs(reducedCornerTable,visited,paths,-1,startingcorner);
    //println(paths.get(2));
    //fill(blue);sphere(paths.get(1)[1]);
    searchpath = new int[paths.size() - 1];
    for(int i = 1; i < paths.size() - 1; i++){
      //println(paths.get(i));
      searchpath[i] = paths.get(i)[1];
    }
    
    
  }
  void recdfs(int[][] reducedCornerTable, boolean[] visited, ArrayList<Integer[]> paths, int parent, int corner){
    visited[corner] = true;
    Integer[] path = {parent, corner};
    paths.add(path);
    for(int i = 0; i < reducedCornerTable[corner].length; i++){
      if(!visited[reducedCornerTable[corner][i]]){
        recdfs(reducedCornerTable,visited,paths,corner,reducedCornerTable[corner][i]);
      }
    }
    Integer[] path2 = {corner,parent};
    paths.add(path2);
  }
    
    
  
  
 
  

  void drawMouse(){
    pt robot = P(currentLocation, new vec(0,0,30));
    pt mouse = P(Of, new vec(0,0,30));
    fill(white);sphere(Of, 10);
    fill(red);caplet(robot,5,mouse,5);
  }
  void moveRobot(){
    pt originLoc = M.triCircumcenter(originCorner);
    pt destinationLoc = M.triCircumcenter(destinationCorner);
    vec direction = V(originLoc,destinationLoc);
    direction.normalize();
    vec travelVector = V(speed, direction);
    currentLocation = P(currentLocation, travelVector);
  }
  void renderRobot(float radius){
    //print(currentLocation.x);
    fill(blue);pillar(currentLocation,calculateHeight(radius),radius);
  }
  float calculateHeight(float radius){
    float h = volume/(3.14159265359 * radius * radius);
    return h;
  }
  void showCurrentDestination(){
    fill(red);sphere(M.triCircumcenter(destinationCorner),10);
    fill(green);sphere(M.cg(M.l(destinationCorner)),10);
    fill(yellow);sphere(M.cg(M.r(destinationCorner)),10);
  }
  int computeNearestCircumcenterCorner(){
    float dist = 9990;
    int min = -1;
    
    pt comparisonPoint;
    if(followingPath){
      comparisonPoint = M.triCircumcenter(searchpath[pathcounter] * 3);
    } else {
      comparisonPoint = Of;
    }
    
    for(int i = 0; i < adjacentCornerTable[destinationCorner].length;i++){
      if(d(comparisonPoint,M.triCircumcenter(adjacentCornerTable[destinationCorner][i])) < dist){
        dist = d(comparisonPoint,M.triCircumcenter(adjacentCornerTable[destinationCorner][i]));
        min = adjacentCornerTable[destinationCorner][i];
      }
    }
    return min;
  }
  float computeNearestCornerDistance(){
    pt dest = M.G[M.V[destinationCorner]];
    pt origin = M.G[M.V[originCorner]];
    pt left = M.G[M.V[M.p(destinationCorner)]];
    pt right = M.G[M.V[M.n(destinationCorner)]];
    
    pt[] arr = new pt[4];
    arr[0] = dest;
    arr[1] = origin;
    arr[2] = left;
    arr[3] = right;
    float dist = 99999;
    for(int i = 0; i < arr.length; i++){
      float distance = d(currentLocation, arr[i]);
      if(d(currentLocation, arr[i]) < dist){
        dist =distance;
      }
    }
    return dist - rb;
  }
    
    
  void showCurrentOrigin(){
    fill(blue);sphere(M.triCircumcenter(originCorner),10);
  }
  boolean atPoint(pt destination){
    if(d(currentLocation, destination) < 5.0 ){
      //println("COLLISION" + collisions);
      //collisions++;
      
      currentLocation = destination;
      return true;
    } else{
      return false;
    }
  }
  int computeCurrentCorner(){
    pt lBound = M.G[M.V[M.p(originCorner)]];
    pt rBound = M.G[M.V[M.n(originCorner)]];
    
    vec vec1 = V(lBound,rBound);
    vec vec2 = V(lBound, currentLocation);
    
    if(det2(vec1,vec2) < 0){
      return originCorner;
    } else {
      return destinationCorner;
    }
  }
  void drawVisited(){
    fill(cyan);
    translate(0,0,4);
    for (int c=0; c<M.nc; c+=3){
      if(visited.get(c)){
        show(M.g(c), M.g(c+1), M.g(c+2));
      }
    }
    translate(0,0,0);
    noFill();
  }
  
  void touchPillars(float rad){
    pt destination = M.G[M.V[destinationCorner]];
    pt back = M.G[M.V[M.o(destinationCorner)]];
    pt left = M.G[M.V[M.n(destinationCorner)]];
    pt right = M.G[M.V[M.p(destinationCorner)]];
    
    pt[] arr = {destination,back,left,right};
    for(int i = 0; i < arr.length; i++){
      if(rad + 5 > d(currentLocation,arr[i]) - rb){
        fill(cyan);pillar(arr[i],columnHeight,rb + 1);
      }
    }
  }
  float[] computeRoots(float a, float b, float c){
    float upper = b*b -(4.0 * a * c);
    if(upper > 0.0){
      float root1 = (-b + sqrt(upper))/(2 * a);
      float root2 = (-b - sqrt(upper))/(2 * a);
      return new float[] {root1, root2};
    } else if(upper == 0){
      float root1 = (-b)/(2 * a);
      return new float[] {root1};
    } else {
      return null;
    }
  }
  float[] checkCollision(pt currentLocation, vec outDirection, pt pillarLocation, float pillarRadius){
    outDirection.normalize();
    vec v2v1 = S(new vec(0,0,0),-1,outDirection);
    float a = dot(v2v1,v2v1);
    
    vec c1c2 = V(currentLocation,pillarLocation);
    float b = 2 * dot(c1c2, v2v1);
    
    float c = dot(c1c2,c1c2) - (pillarRadius * pillarRadius);
    
    float[] roots = computeRoots(a,b,c);
    if(roots == null){
      return null;
    } else {
      return roots;
    }
  }

  void mouseTest(){
    pt loc = P(currentLocation);
    loc.z = 0;
    pt mouse = P(Of);
    mouse.z = 0;
    vec v =  V(loc,mouse);
    v.normalize();
    float min = 9999;
    boolean found = false;
    int hitVertex = -1;
    for(int i = 0; i < M.G.length; i++){
      float[] scal = checkCollision(loc, v, M.G[i], rb);
      if(scal != null){
        
        float mint = min(scal);
        if(mint < min && mint > 0){
          found = true;
          min = mint;
          hitVertex = i;
        }
      }
    }
    if(found){
      fill(white);caplet(loc,10,P(loc,V(min - 10,v)),10);
      fill(yellow);sphere(M.G[hitVertex],100);
    }
  }
  int vectorCollision(vec v){
    pt loc = P(currentLocation);
    loc.z = 0;
    v.normalize();
    float min = 9999;
    boolean found = false;
    int hitVertex = -1;
    for(int i = 0; i < M.G.length; i++){
      float[] scal = checkCollision(loc, v, M.G[i], rb);
      if(scal != null){
        
        float mint = min(scal);
        //check so that -0.5, 1, functions correctly, need to remove negative number
        if(mint < min && mint >= 0){
          found = true;
          min = mint;
          hitVertex = i;
        }
      }
    }
    if(found){
      fill(orange);caplet(loc,3,P(loc,V(min - 10,v)),3);
      fill(white);pillar(M.G[hitVertex],columnHeight/2,rb+5);
      return hitVertex;
    } else {
      //fill(white);caplet(loc,2,P(loc,V(1000,v)),2);
      return -1;
    }
  }
  int[] shootRays(int numRays){
    float radianCircle = 2 * 3.14159265359;
    float increment = radianCircle/numRays;
    ArrayList<Integer> hits = new ArrayList<Integer>();
    for(int i = 0; i < numRays; i++){
      float x = sin(increment*i);
      float y = cos(increment*i);
      vec v = new vec(x,y,0);
      v.normalize();
      int hit = vectorCollision(v);
      if(hit >= 0){
        hits.add(hit);
      }
    }
    int[] hitVertices = new int[hits.size()];
    for(int i = 0; i < hits.size(); i++){
      hitVertices[i] = hits.get(i);
    }
    return hitVertices;
        
  }
  void showSeenPillars(){
    for(int i = 0; i < M.nv; i++){
      if(detectedPillars.get(i)){
        fill(red);pillar(M.G[i],columnHeight * (2/3.0),rb+3);
      }
    }
  }
  
  
}
