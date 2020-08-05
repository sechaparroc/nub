package ik.constraintTest;

import nub.core.Node;
import nub.core.constraint.Hinge;
import nub.core.constraint.SphericalPolygon;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

public class ConstraintHint extends PApplet {
    Scene scene;
    Node n1, n2, n3;

    public void settings(){
        size(800,600, P3D);
    }

    public void setup(){
        scene = new Scene(this);
        scene.setRadius(100);
        scene.fit(1);

        n1 = new Node(pg -> {
            pg.pushStyle();
            pg.fill(255,0,0, 255);
            pg.noStroke();
            pg.box(10);
            pg.popStyle();
        });

        n2 = new Node(pg -> {
            pg.pushStyle();
            pg.fill(0,255,0, 255);
            pg.noStroke();
            pg.sphere(10);
            pg.popStyle();
        });

        n3 = new Node(pg -> {
            pg.pushStyle();
            pg.fill(0,0,255, 255);
            pg.noStroke();
            Scene.drawCylinder(pg,10,20);
            pg.popStyle();
        });

        //Translate nodes
        n2.setReference(n1);
        n3.setReference(n2);

        n2.translate(30, 0,0);
        n3.translate(0, 30,0);

        //Set nodes constraints
        Hinge h1 = new Hinge(radians(40),radians(40));
        h1.setRestRotation(n1.rotation(), n2.translation(), n2.translation().orthogonalVector());
        n1.setConstraint(h1);
        n1.enableHint(Node.CONSTRAINT);

        SphericalPolygon p2 = new SphericalPolygon(radians(20), radians(30));
        p2.setRestRotation(n2.rotation(), n3.translation().orthogonalVector(), n3.translation());
        n2.setConstraint(p2);
        n2.enableHint(Node.CONSTRAINT);

        //disable picking with constraint
        n2.disablePickingMode(Node.CONSTRAINT);

        //Enable bone Hint
        n2.enableHint(Node.BONE, color(255,255,0), 2, true);
        n3.enableHint(Node.BONE, color(0,255,255), 2, false);

        scene.enableHint(Scene.AXES);
    }

    public void draw(){
        lights();
        background(0);
        scene.render();
    }

    @Override
    public void mouseMoved() {
        scene.mouseTag();
    }

    @Override
    public void mouseDragged() {
        if (mouseButton == LEFT)
            scene.mouseSpin();
        else if (mouseButton == RIGHT)
            scene.mouseTranslate();
        else
            scene.scale(scene.mouseDX());
    }

    @Override
    public void mouseWheel(MouseEvent event) {
        scene.moveForward(event.getCount());
    }

    @Override
    public void mouseClicked(MouseEvent event) {
        if (event.getCount() == 2)
            if (event.getButton() == LEFT)
                scene.focus();
            else
                scene.align();
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{"ik.constraintTest.ConstraintHint"});
    }

}
