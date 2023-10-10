from .expert import Expert  # noqa: 401
from .reach_human_expert import ReachHumanExpert, ReachHumanExpertObservation  # noqa: 401
from .reach_human_cart_expert import ReachHumanCartExpert  # noqa: 401
from .pick_place_human_cart_expert import PickPlaceHumanCartExpert, PickPlaceHumanCartExpertObservation  # noqa: 401
from .collaborative_lifting_cart_expert import CollaborativeLiftingCartExpert, CollaborativeLiftingCartExpertObservation  # noqa: 401
from .collaborative_hammering_cart_expert import CollaborativeHammeringCartExpert, CollaborativeHammeringCartExpertObservation  # noqa: 401

REGISTERED_EXPERTS = {
    "ReachHuman": ReachHumanExpert,
    "ReachHumanCart": ReachHumanCartExpert,
    "PickPlaceHumanCart": PickPlaceHumanCartExpert,
    "CollaborativeLiftingCart": CollaborativeLiftingCartExpert,
    "CollaborativeHammeringCart": CollaborativeHammeringCartExpert,
}
