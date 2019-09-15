/**
 * Shadow Mapping.
 * by Jean Pierre Charalambos.
 *
 * Shadow mapping technique ported to nub from Poersch's:
 * https://forum.processing.org/two/discussion/12775/simple-shadow-mapping
 * See also:
 * 1. http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-16-shadow-mapping/
 * 2. https://learnopengl.com/Advanced-Lighting/Shadows/Shadow-Mapping
 *
 * Press 1, 2 and 3 to display the different landscapes.
 * Press ' ' to change the shadow volume from orthographic to perspective.
 * Press 'd' to toggle visual debugging mode.
 */

import nub.primitives.*;
import nub.core.*;
import nub.processing.*;

Scene scene;
Node landscape1, landscape2, landscape3, floor, light;
PShader depthShader;
PShader shadowShader;
PGraphics shadowMap;
float fov = THIRD_PI;
Matrix biasMatrix = new Matrix(
  0.5f, 0.0f, 0.0f, 0.0f, 
  0.0f, 0.5f, 0.0f, 0.0f, 
  0.0f, 0.0f, 0.5f, 0.0f, 
  0.5f, 0.5f, 0.5f, 1.0f
  );
boolean debug;
Graph.Type shadowMapType = Graph.Type.ORTHOGRAPHIC;
float zNear = 10;
float zFar = 600;
int w = 1000;
int h = 1000;

void settings() {
  size(w, h, P3D);
}

void setup() {
  scene = new Scene(this);
  scene.togglePerspective();
  scene.setRadius(max(w, h) / 3);
  scene.fit(1);
  landscape1 = new Node(scene) {
    @Override
    public void graphics(PGraphics pg) {
      if (!isCulled()) {
        float offset = -frameCount * 0.01f;
        pg.fill(0xffff5500);
        for (int z = -5; z < 6; ++z)
          for (int x = -5; x < 6; ++x) {
            pg.pushMatrix();
            pg.translate(x * 12, sin(offset + x) * 20 + cos(offset + z) * 20, z * 12);
            pg.box(10, 100, 10);
            pg.popMatrix();
          }
      }
    }
  };
  landscape1.setPickingThreshold(0);
  landscape2 = new Node(scene) {
    @Override
    public void graphics(PGraphics pg) {
      if (!isCulled()) {
        float angle = -frameCount * 0.0015f, rotation = TWO_PI / 20;
        pg.fill(0xffff5500);
        for (int n = 0; n < 20; ++n, angle += rotation) {
          pg.pushMatrix();
          pg.translate(sin(angle) * 70, cos(angle * 4) * 10, cos(angle) * 70);
          pg.box(10, 100, 10);
          pg.popMatrix();
        }
        pg.fill(0xff0055ff);
        pg.sphere(50);
      }
    }
  };
  landscape2.setPickingThreshold(0);
  landscape2.cull();
  landscape3 = new Node(scene) {
    @Override
    public void graphics(PGraphics pg) {
      if (!isCulled()) {
        float angle = -frameCount * 0.0015f, rotation = TWO_PI / 20;
        pg.fill(0xffff5500);
        for (int n = 0; n < 20; ++n, angle += rotation) {
          pg.pushMatrix();
          pg.translate(sin(angle) * 70, cos(angle) * 70, 0);
          pg.box(10, 10, 100);
          pg.popMatrix();
        }
        pg.fill(0xff00ff55);
        pg.sphere(50);
      }
    }
  };
  landscape3.setPickingThreshold(0);
  landscape3.cull();
  floor = new Node(scene) {
    @Override
    public void graphics(PGraphics pg) {
      pg.fill(0xff222222);
      pg.box(360, 5, 360);
    }
  };
  floor.disableTracking();
  light = new Node(scene) {
    @Override
    public void graphics(PGraphics pg) {
      pg.pushStyle();
      if (debug) {
        pg.fill(0, scene.isTrackedNode(this) ? 255 : 0, 255);
        Scene.drawFrustum(pg, shadowMap, shadowMapType, this, zNear, zFar);
      } else {
        pg.fill(0, 255, 255);
        Scene.drawCone(pg, 150f, 60f, 240f);
      }
      Scene.drawAxes(pg, 300);
      pg.pushStyle();
    }
  };
  light.setPickingThreshold(0);
  light.setMagnitude(400f / 2048f);
  // initShadowPass
  depthShader = loadShader("depth_frag.glsl");
  shadowMap = createGraphics(2048, 2048, P3D);
  shadowMap.shader(depthShader);
  // Testing the appearance of artifacts first
  //shadowMap.noSmooth();

  // initDefaultPass
  shadowShader = loadShader("shadow_frag.glsl", "shadow_vert.glsl");
  shader(shadowShader);
  noStroke();
}

void draw() {
  // 1. Calculate the light position and orientation
  if (!scene.isTrackedNode(light)) {
    float lightAngle = frameCount * 0.002f;
    light.setPosition(sin(lightAngle) * 160, 160, cos(lightAngle) * 160);
  }
  light.setYAxis(Vector.projectVectorOnAxis(light.yAxis(), new Vector(0, 1, 0)));
  light.setZAxis(new Vector(light.position().x(), light.position().y(), light.position().z()));

  // 2. Render the shadowmap from light node 'point-of-view'
  shadowMap.beginDraw();
  shadowMap.noStroke();
  shadowMap.background(0xffffffff); // Will set the depth to 1.0 (maximum depth)
  scene.render(shadowMap, shadowMapType, light, zNear, zFar);
  shadowMap.endDraw();

  // 3. Render the scene from the scene.eye() node
  background(0);
  if (!debug) {
    Matrix projectionView = light.projectionView(shadowMapType, shadowMap.width, shadowMap.height, zNear, zFar);
    Matrix lightMatrix = Matrix.multiply(biasMatrix, projectionView);
    //Scene.setUniform(shadowShader, "shadowTransform", Matrix.multiply(lightMatrix, Matrix.inverse(scene.view())));
    Scene.setUniform(shadowShader, "shadowTransform", Matrix.multiply(lightMatrix, scene.eye().viewInverse()));
    Vector lightDirection = scene.eye().displacement(light.zAxis(false));
    Scene.setUniform(shadowShader, "lightDirection", lightDirection);
    shadowShader.set("shadowMap", shadowMap);
  }
  scene.render();
}

void keyPressed() {
  if (key == '1' || key == '2' || key == '3') {
    landscape1.cull(key != '1');
    landscape2.cull(key != '2');
    landscape3.cull(key != '3');
  } else if (key == ' ') {
    shadowMapType = shadowMapType == Graph.Type.ORTHOGRAPHIC ? Graph.Type.PERSPECTIVE : Graph.Type.ORTHOGRAPHIC;
    light.setMagnitude(shadowMapType == Graph.Type.ORTHOGRAPHIC ? 400f / 2048f : tan(fov / 2));
  } else if (key == 'd') {
    debug = !debug;
    if (debug)
      resetShader();
    else
      shader(shadowShader);
  }
}

void mouseMoved() {
  scene.cast();
}

void mouseDragged() {
  if (mouseButton == LEFT)
    scene.spin();
  else if (mouseButton == RIGHT)
    scene.translate();
  else
    scene.moveForward(mouseX - pmouseX);
}

void mouseWheel(MouseEvent event) {
  if (event.isShiftDown()) {
    int shift = event.getCount() * 20;
    if (zFar + shift > zNear)
      zFar += shift;
  } else
    scene.scale(event.getCount() * 20);
}