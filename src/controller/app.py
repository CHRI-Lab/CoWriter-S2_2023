from flask import Flask, request, jsonify
from .nao_controller import NaoSettings

nao_settings = NaoSettings()
app = Flask(__name__)

@app.route('/get_settings', methods=['POST'])
def get_settings():
    return jsonify(nao_settings.get_settings())

@app.route('/set_interaction', methods=['POST'])
def set_interaction():
    nao_settings.set_interaction()
    return jsonify({'status': 'success'})

@app.route('/speak_and_log_phrases', methods=['POST'])
def nao_speak_and_log_phrase():
    data = request.get_json()
    nao_settings.speak_and_log_phrase(data.get("phrase"))
    return jsonify({'status': 'success'})

@app.route('/look_at_tablet', methods=['POST'])
def look_at_tablet():
    nao_settings.look_at_tablet()
    return jsonify({'status': 'success'})

@app.route('/look_and_ask_for_feedback', methods=['POST'])
def look_and_ask_for_feedback():
    data = request.get_json()
    nao_settings.look_and_ask_for_feedback(data.get("phrase"), data.get("side"))
    return jsonify({'status': 'success'})

@app.route('/handle_look_and_ask_for_feedback', methods=['POST'])
def handle_look_and_ask_for_feedback():
    data = request.get_json()
    nao_settings.handle_look_and_ask_for_feedback(data.get("phrase"))
    return jsonify({'status': 'success'})

@app.route('/nao_rest', methods=['GET'])
def nao_rest():
    nao_settings.nao_rest()
    return jsonify({'status': 'success'})

@app.route('/say', methods=['POST'])
def say():
    data = request.get_json()
    nao_settings.text_to_speech.say(data.get("phrase"))
    return jsonify({'status': 'success'})

@app.route('/rest', methods=['POST'])
def rest():
    nao_settings.posture_proxy.motion_proxy.rest()
    return jsonify({'status': 'success'})

@app.route('/set_stiffness', methods=['POST'])
def set_stiffness():
    data = request.get_json()
    nao_settings.posture_proxy.motion_proxy.setStiffnesses(data.get("joints"), data.get("stiffness"))   
    return jsonify({'status': 'success'})

@app.route('/go_to_posture', methods=['POST'])
def go_to_posture():
    data = request.get_json()
    nao_settings.posture_proxy.goToPosture(data.get("posture"), data.get("speed"))
    return jsonify({'status': 'success'})

@app.route('/open_hand', methods=['POST'])
def open_hand():
    data = request.get_json()
    nao_settings.motion_proxy.openHand(data.get("hand"))
    return jsonify({'status': 'success'})

@app.route('/close_hand', methods=['POST'])
def close_hand():
    data = request.get_json()
    nao_settings.motion_proxy.closeHand(data.get("hand"))
    return jsonify({'status': 'success'})

@app.route('/position_interpolation', methods=['POST'])
def position_interpolation():
    data = request.get_json()
    nao_settings.motion_proxy.positionInterpolation(
        data.get("effector"),
        data.get("space"),
        data.get("path"),
        data.get("axisMask"),
        data.get("times"),
        data.get("isAbsolute"),
    )
    return jsonify({'status': 'success'})


if __name__ == '__main__':
    app.run(debug=True, port=5000) 