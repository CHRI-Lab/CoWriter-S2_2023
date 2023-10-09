from flask import Blueprint

CHILD_BP = Blueprint("child", __name__, url_prefix="/child")


# TODO: Difan

# @CHILD_BP.route("/example_route", methods=["POST"])
# def example_route():
# pass
