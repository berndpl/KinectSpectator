// Daniel Shiffman
// Kinect Point Cloud example
// http://www.shiffman.net
// https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing

// Kinect, Minim, OSC
import org.openkinect.*;
import org.openkinect.processing.*;
import ddf.minim.*; 
import oscP5.*; //OSC
import netP5.*; //OSC
import processing.video.*;

// Kinect, Minim, OSC
Kinect kinect;

Minim minim;
AudioInput input;

OscP5 oscP5;

MovieMaker mm;  // Declare MovieMaker object

float v_fader1 = 0.5f;
float v_toggle1 = 0.5f;
float v_toggle2 = 0.5f;
float v_toggle3 = 0.5f;
float v_toggle4 = 0.5f;
float v_xy1_x = 0.5f;
float v_xy1_y = 0.5f;

float a = 3;
float b = 3;
float c = 3;
float g = 1;
float i = 0;
float i_target = 0;

// Size of kinect image
int w = 640;
int h = 480;


// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

void setup() {
  size(800,600,P3D);
  /* start oscP5, listening for incoming messages at port 8000 */
  oscP5 = new OscP5(this,8000);

  // Audiotoolkit anlegen
  minim = new Minim (this);
  input = minim.getLineIn (Minim.STEREO, 512);
  
  /* Kinect*/
  kinect = new Kinect(this);
  kinect.start();
  kinect.enableDepth(true);
  // We don't need the grayscale image in this example
  // so this makes it more efficient
  kinect.processDepthImage(false);

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);

 
  }

 mm = new MovieMaker(this, width, height, "drawing.mov",
                       10, MovieMaker.H263, MovieMaker.LOSSLESS);

}

void oscEvent(OscMessage theOscMessage) {

    String addr = theOscMessage.addrPattern();

    float val = theOscMessage.get(0).floatValue();
    
    if(addr.equals("/1/fader1"))        { v_fader1 = val; }
    else if(addr.equals("/1/toggle1"))  { v_toggle1 = val; }
    else if(addr.equals("/1/toggle2"))  { v_toggle2 = val; }
    else if(addr.equals("/1/toggle3"))  { v_toggle3 = val; }
    else if(addr.equals("/1/toggle4"))  { v_toggle4 = val; }
    else if(addr.equals("/1/xy1"))  { v_xy1_y = theOscMessage.get(1).floatValue(); v_xy1_x = val; }       
    
}

void draw() {

  background(0);
  fill(255);
  textMode(SCREEN);
  text("Kinect FR: " + (int)kinect.getDepthFPS() + "\nProcessing FR: " + (int)frameRate,10,16);
  text("Rotate X: " + (float)a + "\nRotate Y: " + (float)b,10,48);
  text("Scale: " + (float)c,10,80); 
  
  text("Fader 1 (Scale): " + (float)v_fader1,10,110); 
  text("XY (X): " + (float)v_xy1_x,10,130); 
  text("XY (Y): " + (float)v_xy1_y,10,150);   

  text("Distortion: " + (float)g,10,180);   
  text("Input Level: " + (float)i,10,210);   
  text("Input Level (Target): " + (float)i_target,10,240);     
  i = input.mix.level ();
  
  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 4;

  // Translate and rotate
  translate(width/2,height/2,-50);
  a = map(v_xy1_x,0,1,0,13);
  rotateY(a); // -1 - 1.5 
  b = map(v_xy1_y,0,1,0,13);
  rotateX(b); //0 - 6.2
  c = map(v_fader1,0,1,0,6);
  scale(c); //0 - 6

  for(int x=0; x<w; x+=skip) {
    for(int y=0; y<h; y+=skip) {
      int offset = x+y*w;

      // Convert kinect data to world yz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x,y,rawDepth);

      stroke(255);
      pushMatrix();
      // Scale up by 200
      float factor = 200;
      translate(v.x*factor,v.y*factor,factor-v.z*factor);
      // Draw a point
      point(0,0);
      popMatrix();
    }
  }
  
  // Rotate
// a += 0.015f;
mm.addFrame();  // Add window's pixels to movie
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  
  i_target =  map(i,0,1,0,10);

  result.x = (float)(((x+(g*i_target*random(1, 6))) - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void keyPressed(){
  if (key == 'g'){
     g += 1;
  }
  if (key == 'h'){
     g -= 1;
  }  
  if (key == 'w'){
     c += 0.02;
  }
  if (key == 's'){
     c -= 0.02;
  } 
  if (key == 'q'){
     b += 0.02;
  }
  if (key == 'a'){
     b -= 0.02;
  } 
  if (key == 'j'){
     a += 0.02;
  }
  if (key == 'k'){
     a -= 0.02;
  } 
  if (key == ' ') {
    mm.finish();  // Finish the movie if space bar is pressed!
  }
}

void stop() {
  kinect.quit();
  super.stop();
}

