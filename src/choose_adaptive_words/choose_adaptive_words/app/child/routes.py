from flask import Blueprint, request, current_app, jsonify

CHILD_BP = Blueprint("child", __name__, url_prefix="/child")


# TODO: Difan


# @CHILD_BP.route("/example_route", methods=["POST"])
# def example_route():
#   return {"status": "ok"}
@CHILD_BP.route("/send_text", methods=["POST"])
def send_Text():
    user_inputs = request.json
    inputText = user_inputs.get("inputText")
    image_url = current_app.child_bridge.call_generate_image(inputText)

    return jsonify({"image_url": image_url})


@CHILD_BP.route("/update_url", methods=["POST"])
def update_url():
    image_url = current_app.child_bridge.image_url
    text_to_wtire = current_app.child_bridge.inputText
    generate_canvas = current_app.child_bridge.generate_canvas
    return jsonify(
        {
            "image_url": image_url,
            "text_to_write": text_to_wtire,
            "generate_canvas": generate_canvas,
        }
    )


@CHILD_BP.route("/send_strokes", methods=["POST"])
def send_strokes():
    user_inputs = request.json
    user_inputs_processed = [
        current_app.child_bridge.process_user_input(user_inputs)
        for user_inputs in user_inputs
    ]
    for strokes in user_inputs_processed:
        current_app.child_bridge.publish_strokesMessage(strokes)
    current_app.child_bridge.update_canvas(False)
    return jsonify({"message": "User input recieved successfully!"})
