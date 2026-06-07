from flask import Flask, render_template_string, request, jsonify
import RPi.GPIO as GPIO

# ── Pin definitions ───────────────────────────────────────────
# Front side - corrected to match your doc
ENA = 12;  IN1 = 17; IN2 = 27    # FR  (Board 32, 11, 13)
ENB = 13;  IN3 = 22; IN4 = 23    # FL  (Board 33, 15, 16)
#testS
# Back side - corrected to match your doc
ENA2 = 18; IN5 = 24; IN6 = 25   # RL  (Board 12, 18, 22)
ENB2 = 19; IN7 = 5;  IN8 = 6    # RR  (Board 35, 29, 31)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
all_pins = [ENA,IN1,IN2,IN3,IN4,ENB,ENA2,IN5,IN6,IN7,IN8,ENB2]
GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)

pwm_fr = GPIO.PWM(ENA,  1000); pwm_fr.start(0)
pwm_fl = GPIO.PWM(ENB,  1000); pwm_fl.start(0)
pwm_rl = GPIO.PWM(ENA2, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(ENB2, 1000); pwm_rr.start(0)

def set_motors(fr_f, fr_b, fl_f, fl_b, rl_f, rl_b, rr_f, rr_b,
               spd_fr=100, spd_fl=100, spd_rl=100, spd_rr=100):
    pwm_fr.ChangeDutyCycle(spd_fr)
    pwm_fl.ChangeDutyCycle(spd_fl)
    pwm_rl.ChangeDutyCycle(spd_rl)
    pwm_rr.ChangeDutyCycle(spd_rr)
    GPIO.output(IN1, fr_f); GPIO.output(IN2, fr_b)
    GPIO.output(IN3, fl_f); GPIO.output(IN4, fl_b)
    GPIO.output(IN5, rl_f); GPIO.output(IN6, rl_b)
    GPIO.output(IN7, rr_f); GPIO.output(IN8, rr_b)

def apply(w, a, s, d, inner=30):
    # columns: FR      FL      RL      RR
    if w and d:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif w and a:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif s and d:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif s and a:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif w:
        set_motors(0,1,  0,1,  0,1,  0,1)
    elif s:
        set_motors(1,0,  1,0,  1,0,  1,0)
    elif a:
        set_motors(0,1,  1,0,  1,0,  0,1)
    elif d:
        set_motors(1,0,  0,1,  0,1,  1,0)
    else:
        set_motors(0,0,  0,0,  0,0,  0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>Car Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #1a1a2e;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100vh;
      font-family: monospace;
      color: white;
      user-select: none;
    }
    h1 { margin-bottom: 10px; font-size: 1.4rem; color: #e94560; }
    #status {
      font-size: 1.1rem;
      margin-bottom: 16px;
      background: #16213e;
      padding: 10px 30px;
      border-radius: 8px;
      min-width: 240px;
      text-align: center;
    }
    .slider-row {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 16px;
      font-size: 0.85rem;
      color: #aaa;
    }
    input[type=range] { width: 140px; accent-color: #e94560; }
    .row { display: flex; gap: 10px; margin: 5px 0; justify-content: center; }
    .key {
      width: 80px; height: 80px;
      background: #16213e;
      border: 2px solid #e94560;
      border-radius: 10px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.8rem;
      cursor: pointer;
      transition: background 0.1s;
      -webkit-tap-highlight-color: transparent;
      touch-action: none;
    }
    .key.active { background: #e94560; }
    #hint { margin-top: 20px; color: #888; font-size: 0.8rem; text-align: center; }
  </style>
</head>
<body>
  <h1>🚗 Car Control</h1>
  <div id="status">■ Stop</div>

  <div class="slider-row">
    <span>Inner wheel speed:</span>
    <input type="range" id="inner-spd" min="0" max="90" value="30">
    <span id="inner-val">30%</span>
  </div>

  <div class="row"><div class="key" id="key-w" data-key="w">▲</div></div>
  <div class="row">
    <div class="key" id="key-a" data-key="a">◄</div>
    <div class="key" id="key-s" data-key="s">▼</div>
    <div class="key" id="key-d" data-key="d">►</div>
  </div>
  <div id="hint">WASD or tap · adjust slider to control turn sharpness</div>

<script>
  const held = new Set();
  let innerSpd = 30;

  document.getElementById('inner-spd').addEventListener('input', function() {
    innerSpd = parseInt(this.value);
    document.getElementById('inner-val').textContent = innerSpd + '%';
  });

  const statusMap = (w,a,s,d) => {
    if (w&&d) return "▲► Forward-Right";
    if (w&&a) return "▲◄ Forward-Left";
    if (s&&d) return "▼► Backward-Right";
    if (s&&a) return "▼◄ Backward-Left";
    if (w)    return "▲  Forward";
    if (s)    return "▼  Backward";
    if (a)    return "↺  Spin Left";
    if (d)    return "↻  Spin Right";
    return     "■  Stop";
  };

  function send() {
    const w = held.has('w'), a = held.has('a'),
          s = held.has('s'), d = held.has('d');
    document.getElementById('status').textContent = statusMap(w,a,s,d);
    ['w','a','s','d'].forEach(k =>
      document.getElementById('key-'+k).classList.toggle('active', held.has(k))
    );
    fetch('/drive', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({w, a, s, d, inner: innerSpd})
    });
  }

  document.addEventListener('keydown', e => {
    if (['w','a','s','d'].includes(e.key) && !held.has(e.key)) {
      held.add(e.key); send();
    }
  });
  document.addEventListener('keyup', e => {
    if (['w','a','s','d'].includes(e.key)) {
      held.delete(e.key); send();
    }
  });

  document.querySelectorAll('.key').forEach(btn => {
    const k = btn.dataset.key;
    const press   = () => { held.add(k);    send(); };
    const release = () => { held.delete(k); send(); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchstart', e => { e.preventDefault(); press();   }, {passive:false});
    btn.addEventListener('touchend',   e => { e.preventDefault(); release(); }, {passive:false});
    btn.addEventListener('touchcancel',e => { e.preventDefault(); release(); }, {passive:false});
  });
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/drive', methods=['POST'])
def drive():
    data = request.get_json()
    apply(data['w'], data['a'], data['s'], data['d'], data.get('inner', 30))
    return jsonify(ok=True)

if __name__ == '__main__':
    try:
        print("Open browser at http://192.168.20.191:5000")
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        set_motors(0,0, 0,0, 0,0, 0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)
        for pwm in [pwm_fr, pwm_fl, pwm_rl, pwm_rr]:
            pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up.")
from flask import Flask, render_template_string, request, jsonify
import RPi.GPIO as GPIO

# ── Pin definitions ───────────────────────────────────────────
ENA = 12; IN1 = 22; IN2 = 23    # FR
ENB = 13; IN3 = 17; IN4 = 27    # FL
ENA2 = 18; IN5 = 5;  IN6 = 6   # RL
ENB2 = 19; IN7 = 24; IN8 = 25  # RR

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
all_pins = [ENA,IN1,IN2,IN3,IN4,ENB,ENA2,IN5,IN6,IN7,IN8,ENB2]
GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)

pwm_fr = GPIO.PWM(ENA,  1000); pwm_fr.start(0)
pwm_fl = GPIO.PWM(ENB,  1000); pwm_fl.start(0)
pwm_rl = GPIO.PWM(ENA2, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(ENB2, 1000); pwm_rr.start(0)

def set_motors(fr_f, fr_b, fl_f, fl_b, rl_f, rl_b, rr_f, rr_b,
               spd_fr=100, spd_fl=100, spd_rl=100, spd_rr=100):
    pwm_fr.ChangeDutyCycle(spd_fr)
    pwm_fl.ChangeDutyCycle(spd_fl)
    pwm_rl.ChangeDutyCycle(spd_rl)
    pwm_rr.ChangeDutyCycle(spd_rr)
    GPIO.output(IN1, fr_f); GPIO.output(IN2, fr_b)
    GPIO.output(IN3, fl_f); GPIO.output(IN4, fl_b)
    GPIO.output(IN5, rl_f); GPIO.output(IN6, rl_b)
    GPIO.output(IN7, rr_f); GPIO.output(IN8, rr_b)

def apply(w, a, s, d, inner=30):
    # columns: FR      FL      RL      RR
    if w and d:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif w and a:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif s and d:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif s and a:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif w:
        set_motors(1,0,  1,0,  1,0,  1,0)
    elif s:
        set_motors(0,1,  0,1,  0,1,  0,1)
    elif a:
        set_motors(1,0,  0,1,  0,1,  1,0)
    elif d:
        set_motors(0,1,  1,0,  1,0,  0,1)
    else:
        set_motors(0,0,  0,0,  0,0,  0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>Car Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #1a1a2e;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100vh;
      font-family: monospace;
      color: white;
      user-select: none;
    }
    h1 { margin-bottom: 10px; font-size: 1.4rem; color: #e94560; }
    #status {
      font-size: 1.1rem;
      margin-bottom: 16px;
      background: #16213e;
      padding: 10px 30px;
      border-radius: 8px;
      min-width: 240px;
      text-align: center;
    }
    .slider-row {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 16px;
      font-size: 0.85rem;
      color: #aaa;
    }
    input[type=range] { width: 140px; accent-color: #e94560; }
    .row { display: flex; gap: 10px; margin: 5px 0; justify-content: center; }
    .key {
      width: 80px; height: 80px;
      background: #16213e;
      border: 2px solid #e94560;
      border-radius: 10px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.8rem;
      cursor: pointer;
      transition: background 0.1s;
      -webkit-tap-highlight-color: transparent;
      touch-action: none;
    }
    .key.active { background: #e94560; }
    #hint { margin-top: 20px; color: #888; font-size: 0.8rem; text-align: center; }
  </style>
</head>
<body>
  <h1>🚗 Car Control</h1>
  <div id="status">■ Stop</div>

  <div class="slider-row">
    <span>Inner wheel speed:</span>
    <input type="range" id="inner-spd" min="0" max="90" value="30">
    <span id="inner-val">30%</span>
  </div>

  <div class="row"><div class="key" id="key-w" data-key="w">▲</div></div>
  <div class="row">
    <div class="key" id="key-a" data-key="a">◄</div>
    <div class="key" id="key-s" data-key="s">▼</div>
    <div class="key" id="key-d" data-key="d">►</div>
  </div>
  <div id="hint">WASD or tap · adjust slider to control turn sharpness</div>

<script>
  const held = new Set();
  let innerSpd = 30;

  document.getElementById('inner-spd').addEventListener('input', function() {
    innerSpd = parseInt(this.value);
    document.getElementById('inner-val').textContent = innerSpd + '%';
  });

  const statusMap = (w,a,s,d) => {
    if (w&&d) return "▲► Forward-Right";
    if (w&&a) return "▲◄ Forward-Left";
    if (s&&d) return "▼► Backward-Right";
    if (s&&a) return "▼◄ Backward-Left";
    if (w)    return "▲  Forward";
    if (s)    return "▼  Backward";
    if (a)    return "↺  Spin Left";
    if (d)    return "↻  Spin Right";
    return     "■  Stop";
  };

  function send() {
    const w = held.has('w'), a = held.has('a'),
          s = held.has('s'), d = held.has('d');
    document.getElementById('status').textContent = statusMap(w,a,s,d);
    ['w','a','s','d'].forEach(k =>
      document.getElementById('key-'+k).classList.toggle('active', held.has(k))
    );
    fetch('/drive', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({w, a, s, d, inner: innerSpd})
    });
  }

  document.addEventListener('keydown', e => {
    if (['w','a','s','d'].includes(e.key) && !held.has(e.key)) {
      held.add(e.key); send();
    }
  });
  document.addEventListener('keyup', e => {
    if (['w','a','s','d'].includes(e.key)) {
      held.delete(e.key); send();
    }
  });

  document.querySelectorAll('.key').forEach(btn => {
    const k = btn.dataset.key;
    const press   = () => { held.add(k);    send(); };
    const release = () => { held.delete(k); send(); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchstart', e => { e.preventDefault(); press();   }, {passive:false});
    btn.addEventListener('touchend',   e => { e.preventDefault(); release(); }, {passive:false});
    btn.addEventListener('touchcancel',e => { e.preventDefault(); release(); }, {passive:false});
  });
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/drive', methods=['POST'])
def drive():
    data = request.get_json()
    apply(data['w'], data['a'], data['s'], data['d'], data.get('inner', 30))
    return jsonify(ok=True)

if __name__ == '__main__':
    try:
        print("Open browser at http://192.168.20.191:5000")
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        set_motors(0,0, 0,0, 0,0, 0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)
        for pwm in [pwm_fr, pwm_fl, pwm_rl, pwm_rr]:
            pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up.")


from flask import Flask, render_template_string, request, jsonify
import RPi.GPIO as GPIO

# ── Pin definitions ───────────────────────────────────────────
# Front side
ENA = 12;  IN1 = 17; IN2 = 27    # FR
ENB = 13;  IN3 = 22; IN4 = 23    # FL

# Back side
ENA2 = 18; IN5 = 24; IN6 = 25   # RL
ENB2 = 19; IN7 = 5;  IN8 = 6    # RR

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
all_pins = [ENA,IN1,IN2,IN3,IN4,ENB,ENA2,IN5,IN6,IN7,IN8,ENB2]
GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)

pwm_fr = GPIO.PWM(ENA,  1000); pwm_fr.start(0)
pwm_fl = GPIO.PWM(ENB,  1000); pwm_fl.start(0)
pwm_rl = GPIO.PWM(ENA2, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(ENB2, 1000); pwm_rr.start(0)

def set_motors(fr_f, fr_b, fl_f, fl_b, rl_f, rl_b, rr_f, rr_b,
               spd_fr=100, spd_fl=100, spd_rl=100, spd_rr=100):
    pwm_fr.ChangeDutyCycle(spd_fr)
    pwm_fl.ChangeDutyCycle(spd_fl)
    pwm_rl.ChangeDutyCycle(spd_rl)
    pwm_rr.ChangeDutyCycle(spd_rr)
    GPIO.output(IN1, fr_f); GPIO.output(IN2, fr_b)
    GPIO.output(IN3, fl_f); GPIO.output(IN4, fl_b)
    GPIO.output(IN5, rl_f); GPIO.output(IN6, rl_b)
    GPIO.output(IN7, rr_f); GPIO.output(IN8, rr_b)

def apply(w, a, s, d, inner=30):
    # columns: FR      FL      RL      RR
    if w and d:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif w and a:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif s and d:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif s and a:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif w:
        set_motors(0,1,  0,1,  0,1,  0,1)
    elif s:
        set_motors(1,0,  1,0,  1,0,  1,0)
    elif a:
        set_motors(0,1,  1,0,  1,0,  0,1)
    elif d:
        set_motors(1,0,  0,1,  0,1,  1,0)
    else:
        set_motors(0,0,  0,0,  0,0,  0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>Car Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #1a1a2e;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100vh;
      font-family: monospace;
      color: white;
      user-select: none;
    }
    h1 { margin-bottom: 10px; font-size: 1.4rem; color: #e94560; }
    #status {
      font-size: 1.1rem;
      margin-bottom: 16px;
      background: #16213e;
      padding: 10px 30px;
      border-radius: 8px;
      min-width: 240px;
      text-align: center;
    }
    .slider-row {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 16px;
      font-size: 0.85rem;
      color: #aaa;
    }
    input[type=range] { width: 140px; accent-color: #e94560; }
    .row { display: flex; gap: 10px; margin: 5px 0; justify-content: center; }
    .key {
      width: 80px; height: 80px;
      background: #16213e;
      border: 2px solid #e94560;
      border-radius: 10px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.8rem;
      cursor: pointer;
      transition: background 0.1s;
      -webkit-tap-highlight-color: transparent;
      touch-action: none;
    }
    .key.active { background: #e94560; }
    #hint { margin-top: 20px; color: #888; font-size: 0.8rem; text-align: center; }
  </style>
</head>
<body>
  <h1>🚗 Car Control</h1>
  <div id="status">■ Stop</div>

  <div class="slider-row">
    <span>Inner wheel speed:</span>
    <input type="range" id="inner-spd" min="0" max="90" value="30">
    <span id="inner-val">30%</span>
  </div>

  <div class="row"><div class="key" id="key-w" data-key="w">▲</div></div>
  <div class="row">
    <div class="key" id="key-a" data-key="a">◄</div>
    <div class="key" id="key-s" data-key="s">▼</div>
    <div class="key" id="key-d" data-key="d">►</div>
  </div>
  <div id="hint">WASD or tap · adjust slider to control turn sharpness</div>

<script>
  const held = new Set();
  let innerSpd = 30;

  document.getElementById('inner-spd').addEventListener('input', function() {
    innerSpd = parseInt(this.value);
    document.getElementById('inner-val').textContent = innerSpd + '%';
  });

  const statusMap = (w,a,s,d) => {
    if (w&&d) return "▲► Forward-Right";
    if (w&&a) return "▲◄ Forward-Left";
    if (s&&d) return "▼► Backward-Right";
    if (s&&a) return "▼◄ Backward-Left";
    if (w)    return "▲  Forward";
    if (s)    return "▼  Backward";
    if (a)    return "↺  Spin Left";
    if (d)    return "↻  Spin Right";
    return     "■  Stop";
  };

  function send() {
    const w = held.has('w'), a = held.has('a'),
          s = held.has('s'), d = held.has('d');
    document.getElementById('status').textContent = statusMap(w,a,s,d);
    ['w','a','s','d'].forEach(k =>
      document.getElementById('key-'+k).classList.toggle('active', held.has(k))
    );
    fetch('/drive', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({w, a, s, d, inner: innerSpd})
    });
  }

  document.addEventListener('keydown', e => {
    if (['w','a','s','d'].includes(e.key) && !held.has(e.key)) {
      held.add(e.key); send();
    }
  });
  document.addEventListener('keyup', e => {
    if (['w','a','s','d'].includes(e.key)) {
      held.delete(e.key); send();
    }
  });

  document.querySelectorAll('.key').forEach(btn => {
    const k = btn.dataset.key;
    const press   = () => { held.add(k);    send(); };
    const release = () => { held.delete(k); send(); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchstart', e => { e.preventDefault(); press();   }, {passive:false});
    btn.addEventListener('touchend',   e => { e.preventDefault(); release(); }, {passive:false});
    btn.addEventListener('touchcancel',e => { e.preventDefault(); release(); }, {passive:false});
  });
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/drive', methods=['POST'])
def drive():
    data = request.get_json()
    apply(data['w'], data['a'], data['s'], data['d'], data.get('inner', 30))
    return jsonify(ok=True)

if __name__ == '__main__':
    try:
        print("Open browser at http://192.168.20.191:5000")
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        set_motors(0,0, 0,0, 0,0, 0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)
        for pwm in [pwm_fr, pwm_fl, pwm_rl, pwm_rr]:
            pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up.")

from flask import Flask, render_template_string, request, jsonify
import RPi.GPIO as GPIO

# ── Pin definitions ───────────────────────────────────────────
ENA = 12; IN1 = 22; IN2 = 23    # FR
ENB = 13; IN3 = 17; IN4 = 27    # FL
ENA2 = 18; IN5 = 5;  IN6 = 6   # RL
ENB2 = 19; IN7 = 24; IN8 = 25  # RR

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
all_pins = [ENA,IN1,IN2,IN3,IN4,ENB,ENA2,IN5,IN6,IN7,IN8,ENB2]
GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)

pwm_fr = GPIO.PWM(ENA,  1000); pwm_fr.start(0)
pwm_fl = GPIO.PWM(ENB,  1000); pwm_fl.start(0)
pwm_rl = GPIO.PWM(ENA2, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(ENB2, 1000); pwm_rr.start(0)

def set_motors(fr_f, fr_b, fl_f, fl_b, rl_f, rl_b, rr_f, rr_b,
               spd_fr=100, spd_fl=100, spd_rl=100, spd_rr=100):
    pwm_fr.ChangeDutyCycle(spd_fr)
    pwm_fl.ChangeDutyCycle(spd_fl)
    pwm_rl.ChangeDutyCycle(spd_rl)
    pwm_rr.ChangeDutyCycle(spd_rr)
    GPIO.output(IN1, fr_f); GPIO.output(IN2, fr_b)
    GPIO.output(IN3, fl_f); GPIO.output(IN4, fl_b)
    GPIO.output(IN5, rl_f); GPIO.output(IN6, rl_b)
    GPIO.output(IN7, rr_f); GPIO.output(IN8, rr_b)

def apply(w, a, s, d, inner=30):
    # columns: FR      FL      RL      RR
    if w and d:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif w and a:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif s and d:
        set_motors(1,0,  0,1,  0,1,  1,0,
                   spd_fr=inner, spd_fl=100, spd_rl=100, spd_rr=inner)
    elif s and a:
        set_motors(0,1,  1,0,  1,0,  0,1,
                   spd_fr=100, spd_fl=inner, spd_rl=inner, spd_rr=100)
    elif w:
        set_motors(0,1,  0,1,  0,1,  0,1)
    elif s:
        set_motors(1,0,  1,0,  1,0,  1,0)
    elif a:
        set_motors(0,1,  1,0,  1,0,  0,1)
    elif d:
        set_motors(1,0,  0,1,  0,1,  1,0)
    else:
        set_motors(0,0,  0,0,  0,0,  0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>Car Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #1a1a2e;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100vh;
      font-family: monospace;
      color: white;
      user-select: none;
    }
    h1 { margin-bottom: 10px; font-size: 1.4rem; color: #e94560; }
    #status {
      font-size: 1.1rem;
      margin-bottom: 16px;
      background: #16213e;
      padding: 10px 30px;
      border-radius: 8px;
      min-width: 240px;
      text-align: center;
    }
    .slider-row {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 16px;
      font-size: 0.85rem;
      color: #aaa;
    }
    input[type=range] { width: 140px; accent-color: #e94560; }
    .row { display: flex; gap: 10px; margin: 5px 0; justify-content: center; }
    .key {
      width: 80px; height: 80px;
      background: #16213e;
      border: 2px solid #e94560;
      border-radius: 10px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.8rem;
      cursor: pointer;
      transition: background 0.1s;
      -webkit-tap-highlight-color: transparent;
      touch-action: none;
    }
    .key.active { background: #e94560; }
    #hint { margin-top: 20px; color: #888; font-size: 0.8rem; text-align: center; }
  </style>
</head>
<body>
  <h1>🚗 Car Control</h1>
  <div id="status">■ Stop</div>

  <div class="slider-row">
    <span>Inner wheel speed:</span>
    <input type="range" id="inner-spd" min="0" max="90" value="30">
    <span id="inner-val">30%</span>
  </div>

  <div class="row"><div class="key" id="key-w" data-key="w">▲</div></div>
  <div class="row">
    <div class="key" id="key-a" data-key="a">◄</div>
    <div class="key" id="key-s" data-key="s">▼</div>
    <div class="key" id="key-d" data-key="d">►</div>
  </div>
  <div id="hint">WASD or tap · adjust slider to control turn sharpness</div>

<script>
  const held = new Set();
  let innerSpd = 30;

  document.getElementById('inner-spd').addEventListener('input', function() {
    innerSpd = parseInt(this.value);
    document.getElementById('inner-val').textContent = innerSpd + '%';
  });

  const statusMap = (w,a,s,d) => {
    if (w&&d) return "▲► Forward-Right";
    if (w&&a) return "▲◄ Forward-Left";
    if (s&&d) return "▼► Backward-Right";
    if (s&&a) return "▼◄ Backward-Left";
    if (w)    return "▲  Forward";
    if (s)    return "▼  Backward";
    if (a)    return "↺  Spin Left";
    if (d)    return "↻  Spin Right";
    return     "■  Stop";
  };

  function send() {
    const w = held.has('w'), a = held.has('a'),
          s = held.has('s'), d = held.has('d');
    document.getElementById('status').textContent = statusMap(w,a,s,d);
    ['w','a','s','d'].forEach(k =>
      document.getElementById('key-'+k).classList.toggle('active', held.has(k))
    );
    fetch('/drive', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({w, a, s, d, inner: innerSpd})
    });
  }

  document.addEventListener('keydown', e => {
    if (['w','a','s','d'].includes(e.key) && !held.has(e.key)) {
      held.add(e.key); send();
    }
  });
  document.addEventListener('keyup', e => {
    if (['w','a','s','d'].includes(e.key)) {
      held.delete(e.key); send();
    }
  });

  document.querySelectorAll('.key').forEach(btn => {
    const k = btn.dataset.key;
    const press   = () => { held.add(k);    send(); };
    const release = () => { held.delete(k); send(); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchstart', e => { e.preventDefault(); press();   }, {passive:false});
    btn.addEventListener('touchend',   e => { e.preventDefault(); release(); }, {passive:false});
    btn.addEventListener('touchcancel',e => { e.preventDefault(); release(); }, {passive:false});
  });
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/drive', methods=['POST'])
def drive():
    data = request.get_json()
    apply(data['w'], data['a'], data['s'], data['d'], data.get('inner', 30))
    return jsonify(ok=True)

if __name__ == '__main__':
    try:
        print("Open browser at http://192.168.20.191:5000")
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        set_motors(0,0, 0,0, 0,0, 0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)
        for pwm in [pwm_fr, pwm_fl, pwm_rl, pwm_rr]:
            pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up.")