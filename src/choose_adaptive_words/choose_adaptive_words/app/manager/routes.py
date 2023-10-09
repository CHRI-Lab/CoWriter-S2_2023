from flask import Blueprint, request, current_app

MANAGER_BP = Blueprint("manager", __name__, url_prefix="/manager")


@MANAGER_BP.route("/words_to_write", methods=["POST"])
def words_to_write():
    data = request.get_json()
    current_app.manager_bridge.word_to_write(data["word"])
    return {"status": "ok"}


@MANAGER_BP.route("/erase", methods=["POST"])
def erase():
    data = request.get_json()
    current_app.manager_bridge.word_to_write(data["word"])
    return {"status": "ok"}


@MANAGER_BP.route("/talk_to_me", methods=["POST"])
def talk_to_me():
    current_app.manager_bridge.talk_to_me()
    return {"status": "ok"}


@MANAGER_BP.route("/gpt_text", methods=["POST"])
def gpt_text():
    data = request.get_json()
    current_app.manager_bridge.gpt_text(data["text"])
    return {"status": "ok"}


@MANAGER_BP.route("/learning_pace", methods=["POST"])
def learning_pace():
    data = request.get_json()
    current_app.manager_bridge.learning_pace(data["pace"])
    return {"status": "ok"}


@MANAGER_BP.route("/stop", methods=["POST"])
def stop():
    current_app.manager_bridge.stop()
    return {"status": "ok"}
