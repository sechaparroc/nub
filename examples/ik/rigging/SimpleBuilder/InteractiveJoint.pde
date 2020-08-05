class InteractiveJoint extends Joint {
    protected Vector _desiredTranslation;
    public InteractiveJoint(int red, int green, int blue, float radius) {
        super(red, green, blue, radius);
    }
    public InteractiveJoint(float radius) {
        super(radius);
    }

    @Override
    public void interact(Object... gesture){
        String command = (String) gesture[0];
        if(command.matches("Add")){
            if(_desiredTranslation != null) {
                if(gesture.length == 3)
                    addChild((Scene) gesture[1], (Vector) gesture[2]);
                else
                    addChild((Scene) gesture[1], (Vector) gesture[2], (Skeleton) gesture[3]);
            }
            _desiredTranslation = null;
        } else if(command.matches("OnAdding")){
            _desiredTranslation = translateDesired((Scene) gesture[1],(Vector) gesture[2]);
        } else if(command.matches("Reset")){
            _desiredTranslation = null;
        } else if(command.matches("Remove")){
            if(gesture.length == 1)
                removeChild();
            else
                removeChild((Skeleton) gesture[1]);
        }
    }

    @Override
    public void graphics(PGraphics pg) {
        super.graphics(pg);
        //Draw desired position
        if(!depth)pg.hint(PConstants.DISABLE_DEPTH_TEST);
        if(_desiredTranslation != null){
            pg.pushStyle();
            pg.stroke(0,255,0);
            pg.strokeWeight(_radius/4f);
            pg.line(0,0,0, _desiredTranslation.x()/this.scaling(), _desiredTranslation.y()/this.scaling(), _desiredTranslation.z()/this.scaling());
            pg.popStyle();
        }
        if(!depth)pg.hint(PConstants.ENABLE_DEPTH_TEST);
        pg.pushStyle();
        stroke(255);
        scene.drawBullsEye(this);
        pg.popStyle();
    }

    public void addChild(Scene focus, Vector mouse){
        addChild(focus, mouse, null);
    }

    public void addChild(Scene focus, Vector mouse, Skeleton skeleton){
        InteractiveJoint joint = new InteractiveJoint(this.radius());
        joint.setPickingThreshold(this.pickingThreshold());
        if(skeleton != null){
            skeleton.addJoint("J" + skeleton.joints().size(), joint);
        }
        joint.setReference(this);
        joint.setTranslation(joint.translateDesired(focus, mouse));
    }

    public void removeChild(){
        Graph.prune(this);
    }

    public void removeChild(Skeleton skeleton){
        skeleton.prune(this);
    }

    public Vector desiredTranslation(){
        return _desiredTranslation;
    }

    public Vector translateDesired(Scene scene, Vector point) {
        Vector delta = Vector.subtract(point, scene.screenLocation(position()));
        delta.setZ(0);
        Vector vector = scene.displacement(delta, this);
        return reference() == null ? worldDisplacement(vector) : reference().displacement(vector, this);
    }
}