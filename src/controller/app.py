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
def go_to_posture(posture, speed):
    nao_settings.posture_proxy.goToPosture(posture, speed)
    return jsonify({'status': 'success'})


if __name__ == '__main__':
    app.run(debug=True, port=5000) 