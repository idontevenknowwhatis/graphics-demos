
class pts // class for manipulaitng and displaying pointclouds or polyloops in 3D 
  { 
    int maxnv = 16000;                 //  max number of vertices
    pt[] G = new pt [maxnv];           // geometry table (vertices)
    char[] L = new char [maxnv];             // labels of points
    vec [] LL = new vec[ maxnv];  // displacement vectors
    Boolean loop=true;          // used to indicate closed loop 3D control polygons
    int pv =0,     // picked vertex index,
        iv=0,      //  insertion vertex index
        dv = 0,   // dancer support foot index
        nv = 0,    // number of vertices currently used in P
        pp=1; // index of picked vertex

  pts() {}
  pts declare() 
    {
    for (int i=0; i<maxnv; i++) G[i]=P(); 
    for (int i=0; i<maxnv; i++) LL[i]=V(); 
    return this;
    }     // init all point objects
  pts empty() {nv=0; pv=0; return this;}                                 // resets P so that we can start adding points
  pts addPt(pt P, char c) { G[nv].setTo(P); pv=nv; L[nv]=c; nv++;  return this;}          // appends a new point at the end
  pts addPt(pt P) { G[nv].setTo(P); pv=nv; L[nv]='f'; nv++;  return this;}          // appends a new point at the end
  pts addPt(float x,float y) { G[nv].x=x; G[nv].y=y; pv=nv; nv++; return this;} // same byt from coordinates
  pts copyFrom(pts Q) {empty(); nv=Q.nv; for (int v=0; v<nv; v++) G[v]=P(Q.G[v]); return this;} // set THIS as a clone of Q

  pts resetOnCircle(int k, float r)  // sets THIS to a polyloop with k points on a circle of radius r around origin
    {
    empty(); // resert P
    pt C = P(); // center of circle
    for (int i=0; i<k; i++) addPt(R(P(C,V(0,-r,0)),2.*PI*i/k,C)); // points on z=0 plane
    pv=0; // picked vertex ID is set to 0
    return this;
    } 
  // ********* PICK AND PROJECTIONS *******  
  int SETppToIDofVertexWithClosestScreenProjectionTo(pt M)  // sets pp to the index of the vertex that projects closest to the mouse 
    {
    pp=0; 
    for (int i=1; i<nv; i++) if (d(M,ToScreen(G[i]))<=d(M,ToScreen(G[pp]))) pp=i; 
    return pp;
    }
  pts showPicked() {show(G[pv],23); return this;}
  pt closestProjectionOf(pt M)    // Returns 3D point that is the closest to the projection but also CHANGES iv !!!!
    {
    pt C = P(G[0]); float d=d(M,C);       
    for (int i=1; i<nv; i++) if (d(M,G[i])<=d) {iv=i; C=P(G[i]); d=d(M,C); }  
    for (int i=nv-1, j=0; j<nv; i=j++) { 
       pt A = G[i], B = G[j];
       if(projectsBetween(M,A,B) && disToLine(M,A,B)<d) {d=disToLine(M,A,B); iv=i; C=projectionOnLine(M,A,B);}
       } 
    return C;    
    }

  // ********* MOVE, INSERT, DELETE *******  
  pts insertPt(pt P) { // inserts new vertex after vertex with ID iv
    for(int v=nv-1; v>iv; v--) {G[v+1].setTo(G[v]);  L[v+1]=L[v];}
     iv++; 
     G[iv].setTo(P);
     L[iv]='f';
     nv++; // increments vertex count
     return this;
     }
  pts insertClosestProjection(pt M) {  
    pt P = closestProjectionOf(M); // also sets iv
    insertPt(P);
    return this;
    }
  pts deletePicked() 
    {
    for(int i=pv; i<nv; i++) 
      {
      G[i].setTo(G[i+1]); 
      L[i]=L[i+1]; 
      }
    pv=max(0,pv-1); 
    nv--;  
    return this;
    }
  pts setPt(pt P, int i) { G[i].setTo(P); return this;}
  
  pts drawBalls(float r) {for (int v=0; v<nv; v++) show(G[v],r); return this;}
  pts showPicked(float r) {show(G[pv],r); return this;}
  pts drawClosedCurve(float r) 
    {
    fill(black);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    fill(magenta);
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    pushMatrix(); //translate(0,0,1); 
    scale(1,1,0.03);  
    fill(grey);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    popMatrix();
    return this;
    }
  pts set_pv_to_pp() {pv=pp; return this;}
  pts movePicked(vec V) { G[pv].add(V); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts setPickedTo(pt Q) { G[pv].setTo(Q); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts moveAll(vec V) {for (int i=0; i<nv; i++) G[i].add(V); return this;};   
  pt Picked() {return G[pv];} 
  pt Pt(int i) {if(0<=i && i<nv) return G[i]; else return G[0];} 

  // ********* I/O FILE *******  
 void savePts(String fn) 
    {
    String [] inppts = new String [nv+1];
    int s=0;
    inppts[s++]=str(nv);
    for (int i=0; i<nv; i++) {inppts[s++]=str(G[i].x)+","+str(G[i].y)+","+str(G[i].z)+","+L[i];}
    saveStrings(fn,inppts);
    };
  
  void loadPts(String fn) 
    {
    println("loading: "+fn); 
    String [] ss = loadStrings(fn);
    String subpts;
    int s=0;   int comma, comma1, comma2;   float x, y;   int a, b, c;
    nv = int(ss[s++]); print("nv="+nv);
    for(int k=0; k<nv; k++) 
      {
      int i=k+s; 
      //float [] xy = float(split(ss[i],",")); 
      String [] SS = split(ss[i],","); 
      G[k].setTo(float(SS[0]),float(SS[1]),float(SS[2]));
      L[k]=SS[3].charAt(0);
      }
    pv=0;
    };
 
  // Dancer
  void setPicekdLabel(char c) {L[pp]=c;}
  


  void setFifo() 
    {
    _LookAtPt.reset(G[dv],60);
    }              


  void next() {dv=n(dv);}
  int n(int v) {return (v+1)%nv;} //next1
  int p(int v) {if(v==0) return nv-1; else return v-1;}
  
  int n(int v, int m) {
    int loc = v;
    for(int i = 0; i < m; i++){
      loc = n(loc);
    }
    return loc;
  }
  int p(int v,int m) {
    int loc = v;
    for(int i = 0; i < m; i++){
      loc = p(loc);
    }
    return loc;
}

int index(int v, int m){
  if(m < 0){
    return p(v, abs(m));
  } else {
    return n(v,abs(m));
  }
}
  
  pts subdivideDemoInto(pts Q) 
    {
    Q.empty();
    for(int i=0; i<nv; i++)
      {
      Q.addPt(P(G[i])); 
      Q.addPt(P(G[i],G[n(i)])); 
      //...
      }
    return this;
    }  
  pt selectPoint(int j, int offset, pts array){
    int result = j + offset;
    if(result < 0){
      return array.G[nv + result];
    } else {
       return array.G[result % array.nv];
    }
  }
  pt selectPoint(int j, int offset, pt[] array,int nv){
    int result = j + offset;
    if(result < 0){
      return array[nv + result];
    } else {
       return array[result % nv];
    }
  }
  pts subdivideQuinticInto(pts points){
    points.empty();
    /*
    for(int i = 0; i < nv; i++){
      float constant = 1.5;
      float a = constant;
      pt A = selectPoint(i,-1,Q);
      float b = (8 - 2* constant);
      pt B = selectPoint(i,0,Q);
      float c = constant;
      pt C = selectPoint(i,1,Q);
      pt q = P(a,A,b,B,c,C);
      points.addPt(P(1.0/8,q));
      
      float x = constant - 1;
      pt X = selectPoint(i, -1, Q);
      float y = 9 - constant;
      pt Y = selectPoint(i, 0, Q);
      float z = 9 - constant;
      pt Z = selectPoint(i, 1, Q);
      float w = constant - 1;
      pt W = selectPoint(i,2,Q);
      pt q2 = P(x,X,y,Y,z,Z,w,W);
      points.addPt(P(1.0/16,q2));
    }
    */
    for(int i = 0; i < nv; i++){
      float constant = 1.5;
      float a = constant;
      pt A = selectPoint(i,-1,Q);
      float b = (8 - 2* constant);
      pt B = selectPoint(i,0,Q);
      float c = constant;
      pt C = selectPoint(i,1,Q);
      pt q = P(a,A,b,B,c,C);
      points.addPt(P(1.0/8,q));
      
      float x = constant - 1;
      pt X = selectPoint(i, -1, Q);
      float y = 9 - constant;
      pt Y = selectPoint(i, 0, Q);
      float z = 9 - constant;
      pt Z = selectPoint(i, 1, Q);
      float w = constant - 1;
      pt W = selectPoint(i,2,Q);
      pt q2 = P(x,X,y,Y,z,Z,w,W);
      points.addPt(P(1.0/16,q2));
    }
    return this;
  }
  int duration = 18;
  int lFrame = 0;
  int lOffset = - (duration)/3;
  
  int rFrame = duration/2;
  int rOffset = 0;
  
  int lHandFrame = 8;
  int rHandFrame = 0;
  
  float frameToRadians = 3.14159265359 / (duration/3);
  float handFrameToRadians = 2 * 3.14159265359/ duration;
  
  void displaySkater() 
      {
      if(showCurve) {
      fill(green); for (int j=0; j<nv; j++) caplet(G[j],3,G[n(j)],3); 
      }  
      pt[] B = new pt [nv];           // geometry table (vertices)
      for (int j=0; j<nv; j++){
        
        pt A = selectPoint(j,-1,G,nv);
        pt Bp = G[j];
        pt C = selectPoint(j,1,G,nv);
        
        vec bc = V(Bp,C);
        vec ba = V(Bp,A);
        vec accel = A(bc,ba);
        accel.add(new vec(0,0,-100));
        float scalefactor = -0.5;
        for(int i = 0; i < level; i++){
          scalefactor*= 3.5;
        }
        
        
        vec foot = V(V(0,0,-100), scalefactor,accel);
        
        B[j] = P(G[j],foot);
        
        
      };
      if(showPath) {fill(lime); for (int j=0; j<nv; j++) caplet(B[j],6,B[n(j)],6);} 
      if(showPath) {fill(cyan); for (int j=0; j<nv; j+=4) arrow(B[j],G[j],3);}
      
      if(animating) f=n(f);
      //produce ds step size with 0,0,1 vertical vector
      if(true) 
        {
        // ....        
                
        //fill(blue);sphere(B[f],10);
        int renderFrameL;
        if(lFrame < duration/3){
          lOffset +=2;
           renderFrameL = index(f,lOffset);
        } else {
          lOffset --;
           renderFrameL = index(f,lOffset);
        }
        lFrame = (lFrame +1)% duration;
        
        
        
        int renderFrameR;
        if(rFrame < duration/3){
          rOffset +=2;
           renderFrameR = index(f,rOffset);
          
        } else {
          rOffset --;
           renderFrameR = index(f,rOffset);
          
        }
        rFrame = (rFrame +1)% duration;
        
        
        vec forwardDirection = V(G[p(f)],G[f]);
        forwardDirection.normalize();
        forwardDirection = V(5,forwardDirection);
        vec up = new vec(0,0,1);
        up.normalize();
        vec offsetVector = cross(forwardDirection,up);
        
        int ds = 3;
        vec lOffset = V(ds,offsetVector);
        vec rOffset = V(-ds,offsetVector);
        
        pt lFoot = P(B[renderFrameL],lOffset);
        pt rFoot = P(B[renderFrameR],rOffset);
        float lFootY = 0;
        float rFootY = 0;
        
        float raiseHeight = 50;
        if(lFrame < duration /3){
          lFootY = raiseHeight * sin(lFrame * frameToRadians);
        }
        if(rFrame < duration/3){
          rFootY = raiseHeight * sin(rFrame * frameToRadians);
        }
        vec lFootYVector = new vec(0,0,lFootY);
        vec rFootYVector = new vec(0,0,rFootY);
        lFoot = P(lFoot,lFootYVector);
        rFoot = P(rFoot,rFootYVector);
        
        
        vec spineOffset = V(B[f],G[f]);
        vec hipOffset = cross(forwardDirection,spineOffset);
        hipOffset.normalize();
        
        pt lHip = P(G[f],V(10,hipOffset));
        pt rHip = P(G[f],V(-10,hipOffset));
        pt hip = new pt(G[f].x,G[f].y,G[f].z);
        
        
        spineOffset.normalize();
        spineOffset = V(70,spineOffset);
        pt spineMid = P(hip,spineOffset);
        
        
        forwardDirection.normalize();
        vec y = forwardDirection;
        spineOffset.normalize();
        vec z = spineOffset;
        vec x = cross(y, z);
        float lExtensionLength = d(lHip,lFoot);
        float rExtensionLength = d(rHip,rFoot);
        float footLength = 110;
        if(lExtensionLength > footLength){
          footLength = lExtensionLength + 1;
        }
        if(rExtensionLength > footLength){
          footLength = rExtensionLength + 1;
        }
        float thighLength = footLength/2;
        float lHalf = lExtensionLength /2;
        float rHalf = rExtensionLength/2;
        
       float lKneeDistance = sqrt((thighLength * thighLength) - (lHalf * lHalf));
       float rKneeDistance = sqrt((thighLength * thighLength) - (rHalf * rHalf));
       if(!inverseKinematics){
         float lLegLength = d(lHip,lFoot);
         float rLegLength = d(rHip,rFoot);
         float lThighLength = (2 * lLegLength)/3;
         float rThighLength = (2 * rLegLength)/3;
         lKneeDistance = sqrt((lThighLength * lThighLength) - ((lLegLength/2) * (lLegLength/2)));
         rKneeDistance = sqrt((rThighLength * rThighLength) - ((rLegLength/2) * (rLegLength/2)));
       }
       vec lKneeOffsetY = V(lHip,lFoot);
       vec rKneeOffsetY = V(rHip,rFoot);
       vec lKneeOffsetX = cross(x,lKneeOffsetY);
       vec rKneeOffsetX = cross(x,rKneeOffsetY);
       lKneeOffsetX.normalize();
       rKneeOffsetX.normalize();
       lKneeOffsetX = V(lKneeDistance,lKneeOffsetX);
       rKneeOffsetX = V(rKneeDistance,rKneeOffsetX);
       lKneeOffsetY = V(0.5,lKneeOffsetY);
       rKneeOffsetY = V(0.5,rKneeOffsetY);
       pt lKnee = P(lHip,lKneeOffsetY);
       pt rKnee = P(rHip,rKneeOffsetY);
       lKnee = P(lKnee,lKneeOffsetX);
       rKnee = P(rKnee,rKneeOffsetX);
       
         
        
        
        float shoulderScale = 40;
        vec lShoulderOffset = V(shoulderScale,x);
        lShoulderOffset = V(lShoulderOffset,V(-10,y));
        vec rShoulderOffset = V(-shoulderScale,x);
        rShoulderOffset = V(rShoulderOffset,V(-10,y));
        pt lShoulder = P(spineMid,lShoulderOffset);
        pt rShoulder = P(spineMid,rShoulderOffset);
        
        //caplet(lHip,3,lFoot,3);
        
        //caplet(rHip,3,rFoot,3);
        
        //caplet(G[f],2,P(G[f],V(100,y)),2);
        //caplet(G[f],2,P(G[f],V(100,x)),2);
        //caplet(G[f],2,P(G[f],V(100,z)),2);
        
        float radius = 10;
        
        float distanceToCenter = 50;
        float armLength = distanceToCenter + radius + 10;
        float elbowLength = armLength/2;
        
        vec handOffsetY = V(3,y);
        vec handOffsetZ = V(-3,z);
        vec handOffset = V(handOffsetY,handOffsetZ);
        handOffset.normalize();
        pt lHandCenter = P(lShoulder,V(distanceToCenter,handOffset));
        pt rHandCenter = P(rShoulder,V(distanceToCenter,handOffset));
        //fill(blue);sphere(lHandCenter,3);
        //fill(red);sphere(rHandCenter,3);
        
        float ZPos = sin(-lHandFrame * handFrameToRadians)/2;
        float YPos = cos(lHandFrame * handFrameToRadians);
        float rZPos = sin(-rHandFrame * handFrameToRadians)/2;
        float rYPos = cos(rHandFrame * handFrameToRadians);
        
        vec lHandZVec = V(ZPos,z);
        vec lHandYPos = V(YPos,y);
        
        vec rHandZVec = V(rZPos,z);
        vec rHandYPos = V(rYPos,y);
        
        pt lHand = P(lHandCenter,V(radius,lHandZVec));
        lHand = P(lHand,V(radius,lHandYPos));
        
        pt rHand = P(rHandCenter,V(radius,rHandZVec));
        rHand = P(rHand,V(radius,rHandYPos));
        
        
        float lHandExtension = d(lShoulder,lHand);
        float lhand2 = lHandExtension/2;
        
        float rHandExtension = d(rShoulder,rHand);
        float rHand2 = rHandExtension/2;
        
        float lElbowOffsetMagnitude = sqrt((elbowLength * elbowLength) - (lhand2 * lhand2));
        float rElbowOffsetMagnitude = sqrt((elbowLength * elbowLength) - (rHand2 * rHand2));
        
        vec lShoulderOffsetY = V(lShoulder,lHand);
        vec lShoulderOffsetX = cross(x,lShoulderOffsetY);
        
        vec rShoulderOffsetY = V(rShoulder,rHand);
        vec rShoulderOffsetX = cross(x,rShoulderOffsetY);
        
        lShoulderOffsetX.normalize();
        rShoulderOffsetX.normalize();
        
        lShoulderOffsetX = V(-lElbowOffsetMagnitude,lShoulderOffsetX);
        rShoulderOffsetX = V(-rElbowOffsetMagnitude,rShoulderOffsetX);
        
        lShoulderOffsetY = V(0.5,lShoulderOffsetY);
        rShoulderOffsetY = V(0.5,rShoulderOffsetY);
        
        pt lElbow = P(lShoulder,lShoulderOffsetY);
        pt rElbow = P(rShoulder,rShoulderOffsetY);
        
        lElbow = P(lElbow,lShoulderOffsetX);
        rElbow = P(rElbow,rShoulderOffsetX);
        
        pt pelvis = P(G[f],V(5,z));
        pt stomach = P(pelvis,V(20,z));
        pt chest = P(spineMid,V(-30,z));
        pt chin = P(spineMid,V(20,z));
        pt head = P(chin,V(10,z));
        
        pt lToe = P(lFoot,V(12,y));
        pt rToe = P(rFoot,V(12,y));
        
        if(showSkater && !showWalkerConstruction){
          fill(yellow);sphere(spineMid,8);
          fill(yellow);sphere(pelvis,10);
          fill(yellow);sphere(stomach,8);
          caplet(pelvis,8,stomach,6);
          caplet(stomach,6,spineMid,8);
          
          fill(green); sphere(lHip,8);
          fill(green); sphere(rHip,8);
          fill(yellow);
          fill(blue);sphere(lKnee,6);
          fill(red);sphere(rKnee,6);
          fill(blue);
          caplet(lHip,8,lKnee,6);
          caplet(lKnee,6,lFoot,4);
          fill(red);
          caplet(rHip,8,rKnee,6);
          caplet(rKnee,6,rFoot,4);
          fill(blue);sphere(lFoot,6);
          fill(red);sphere(rFoot,6);
          
          fill(blue); sphere(lToe,3);
          fill(red); sphere(rToe,3);
          fill(blue); caplet(lFoot,6,lToe,3);
          fill(red);caplet(rFoot,6,rToe,3);
          
          fill(blue); sphere(lShoulder,7);
          fill(red); sphere(rShoulder,7);
          
          fill(yellow);caplet(lShoulder,5,rShoulder,5);
          
          fill(yellow);caplet(chest,5,P(lShoulder,V(-6,x)),5);
          fill(yellow);caplet(chest,5,P(rShoulder,V(6,x)),5);
          
          fill(yellow);caplet(spineMid,4,chin,4);
          fill(yellow);sphere(chin,8);
          fill(yellow);sphere(head,10);
          caplet(chin,8,head,10);
          
          
  
          fill(blue);sphere(lHand,4);
          fill(blue);sphere(lElbow,5);
          caplet(lShoulder,7,lElbow,5);
          caplet(lElbow,5,lHand,4);
          
          fill(red);sphere(rHand,4);
          fill(red);sphere(rElbow,5);
          caplet(rShoulder,7,rElbow,5);
          caplet(rElbow,5,rHand,4);
        
        }else if(showSkater && showWalkerConstruction){
          fill(red); sphere(hip,10);
          fill(red); sphere(spineMid,15);
          caplet(hip,10,spineMid,15);
          fill(brown); sphere(P(B[f],lOffset),5);
          fill(brown); sphere(P(B[f],rOffset),5);
          fill(green); sphere(lFoot,8);
          fill(blue);sphere(rFoot,8);
          fill(red);sphere(rHip,8);
          fill(red);sphere(lHip,8);
          fill(green);caplet(lHip,8,lFoot,8);
          fill(blue);caplet(rHip,8,rFoot,8);
          
        }else{fill(red); arrow(B[f],G[f],20);}
        lHandFrame = (lHandFrame + 1) % duration;
        rHandFrame = (rHandFrame + 1) % duration;
        
        
        
        }
        
        
       //
      }

        

} // end of pts class
