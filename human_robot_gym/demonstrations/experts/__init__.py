from .expert import Expert  # noqa: 401
from .pick_place_expert import PickPlaceExpert, PickPlaceExpertObservation  # noqa: 401

REGISTERED_EXPERTS = {
    "PickPlace": PickPlaceExpert,
}
