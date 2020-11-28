class BaseControl extends Node {
    int _color;
    float _left = 74, _right = 0, _up = 0, _down = 32;
    float _pleft = 74, _pright = 0, _pup = 0, _pdown = 32;
    Vector _initial, _end;
    boolean _modified = false;
    float _max;
    float _max_tan;
    float _height;
    Scene _scene;

    public BaseControl(Scene scene, int col) {
      super();
      _scene = scene;
      _color = col;
      setBullsEyeSize(0);
      setHighlight(0);
      _max = _scene.radius() * 0.8f;
      _max_tan = tan(radians(70));
      _height = _max / _max_tan;
    }

    public boolean modified() {
      return _modified;
    }

    public void setModified(boolean modified) {
      _modified = modified;
    }

    public void update(float tl, float tr, float tu, float td) {
      _left = _pleft = _height * tan(tl);
      _right = _pright = _height * tan(tr);
      _up = _pup = _height * tan(tu);
      _down = _pdown = _height * tan(td);
    }

    public float toAngle(float l) {
      return atan2(l, _height);
    }

    public float left() {
      return _left;
    }

    public float right() {
      return _right;
    }

    public float up() {
      return _up;
    }

    public float down() {
      return _down;
    }

    @Override
    public void graphics(PGraphics pg) {
      pg.pushStyle();
      //Draw base according to each radius
      pg.fill(_color, _scene.node() == this ? 255 : 100);
      pg.noStroke();
      drawCone(pg, 64, 0, 0, 0, _left, _up, _right, _down, false);
      //draw semi-axes
      pg.fill(255, 0, 0);
      pg.stroke(255, 0, 0);
      pg.strokeWeight(3);
      pg.line(0, 0, -_left, 0);
      pg.line(0, 0, _right, 0);
      pg.ellipse(-_left, 0, 3, 3);
      pg.ellipse(_right, 0, 3, 3);

      pg.fill(0, 255, 0);
      pg.stroke(0, 255, 0);
      pg.line(0, 0, 0, _up);
      pg.line(0, 0, 0, -_down);
      pg.ellipse(0, _up, 3, 3);
      pg.ellipse(0, -_down, 3, 3);

      pg.fill(0);
      pg.stroke(0);
      pg.ellipse(0, 0, 3, 3);

      if (_initial != null && _end != null) {
        pg.stroke(255);
        pg.line(_initial.x(), _initial.y(), _end.x(), _end.y());
        pg.fill(255, 0, 0);
        pg.noStroke();
        pg.ellipse(_initial.x(), _initial.y(), 5, 5);
        pg.ellipse(_end.x(), _end.y(), 5, 5);
        pg.fill(255);
      }

      _scene.beginHUD();
      Vector l = _scene.screenLocation(this.worldLocation(new Vector(-_left, 0)));
      Vector r = _scene.screenLocation(this.worldLocation(new Vector(_right, 0)));
      Vector u = _scene.screenLocation(this.worldLocation(new Vector(0, _up)));
      Vector d = _scene.screenLocation(this.worldLocation(new Vector(0, -_down)));

      pg.fill(0);
      pg.textFont(font, 16);
      pg.textAlign(RIGHT, CENTER);
      pg.text("Left", l.x() - 5, l.y());
      pg.textAlign(LEFT, CENTER);
      pg.text("Right", r.x() + 5, r.y());
      pg.textAlign(CENTER, TOP);
      pg.text("Down", u.x(), u.y());
      pg.textAlign(CENTER, BOTTOM);
      pg.text("Up", d.x(), d.y());

      _scene.endHUD();
      pg.popStyle();
    }

    @Override
    public void interact(Object[] gesture) {
      String command = (String) gesture[0];
      if (command.matches("Scale")) {
        if (_initial != null && _end != null) {
          //scale
          scale();
          _modified = true;
        }
        _initial = null;
        _end = null;
      } else if (command.matches("OnScaling")) {
        if (_initial == null) {
          //Get initial point
          _initial = _scene.location((Vector) gesture[1], this);
          _pleft = _left;
          _pright = _right;
          _pdown = _down;
          _pup = _up;
        } else {
          //Get final point
          _end = _scene.location((Vector) gesture[1], this);
          scale();
        }
      } else if (command.matches("Clear")) {
        _initial = null;
        _end = null;
      }
    }

    public void scale() {
      float horizontal = _end.x() - _initial.x();
      float vertical = _end.y() - _initial.y();
      //determine Which radius to scale
      if (_initial.x() > 0) {
        //Scale right radius
        _right = _pright + horizontal;
        //Clamp
        _right = max(min(_scene.radius(), _right), 5);
      } else {
        _left = _pleft - horizontal;
        //Clamp
        _left = max(min(_scene.radius(), _left), 5);
      }
      if (_initial.y() > 0) {
        //Scale right radius
        _up = _pup + vertical;
        //Clamp
        _up = max(min(_scene.radius(), _up), 5);
      } else {
        _down = _pdown - vertical;
        //Clamp
        _down = max(min(_scene.radius(), _down), 5);
      }
    }
  }
