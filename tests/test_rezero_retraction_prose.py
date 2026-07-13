"""Issue #43's retraction, enforced in the prose an OPERATOR actually reads.

``arm_spec._REZERO_UNNECESSARY`` was withdrawn and replaced by
``_REZERO_ARC_UNKNOWN`` because hardware contradicted it. But a retraction that
lives only in the table it corrects is not a retraction: the same false claim was
still being told to the operator's face by ``learn``, by ``explain arm rezero``,
and by ``arm rezero --help``.

The claim, verbatim, as it used to be told
==========================================
    "Only elbow_flex wraps inside its travel."
    "The other four — unnecessary. Their encoders do not wrap inside their travel."

Probed by feel (issue #43), ``shoulder_lift``, ``gripper`` and ``shoulder_pan``
each reached their commandable bound with **no contact** — still physically free —
2, 3 and 11 raw ticks from the seam, and ``shoulder_lift`` then sagged *through*
the seam under gravity with its torque off. At the factory offset the commandable
bound sits one tick below the seam, so the arm was reporting the *seam* as its
boundary and nothing could see past it.

The honest line is **"UNKNOWN, not unnecessary"** — and these tests pin BOTH
halves of that, because replacing one over-claim with the opposite over-claim
would be the same bug wearing different clothes:

* the retracted claim is gone from every operator-facing surface; **and**
* nothing now claims the opposite either (that those joints *do* wrap).

``wrist_roll``'s refusal used to be described here as "the one that is PROVEN, and it
stays". It was not, and it did not. Issue #43 withdrew that too: its 400 contact threshold
sat above the 272/288 its walls can push, so contact could never fire, and the "free range
[21, 4073]" behind the claim was a joint driving into two real walls unheard. It has an
unreachable arc; the arc is 209 ticks; it is refused for being too NARROW to hold a seam,
not for being absent. Same answer, earned honestly — and its claim is retracted here on the
same terms as the others.

And the surfaces RENDER the wording from ``arm_spec`` rather than restating it, so
the prose cannot drift from the table a second time. That is what is asserted here:
not that three files happen to contain the right sentence today, but that they
cannot contain a different one tomorrow.
"""

from __future__ import annotations

import re

from arm101.cli import _build_parser
from arm101.cli._commands import learn
from arm101.explain import catalog
from arm101.hardware import arm_spec

#: The claim hardware withdrew, in every shape it was ever shipped in. Whitespace is
#: normalised before the search because the originals were wrapped across lines.
RETRACTED = (
    "only elbow_flex wraps inside its travel",
    "their encoders do not wrap inside their travel",
    "the other four — unnecessary",
    "the other four are refused because they never wrap",
    # ...and wrist_roll's, withdrawn on the same issue for the same kind of reason: a
    # measurement that was really an instrument's blind spot. It has two real walls.
    "it turns freely all the way round",
    "found no wall anywhere in wrist_roll's travel",
    "this refusal is proven and permanent",
)

#: Words that mark an occurrence of the claim as a **withdrawal of it** rather than an
#: assertion of it. The retraction has to be able to QUOTE what it is retracting —
#: ``arm_spec._REZERO_ARC_UNKNOWN`` does exactly that ("This message used to say ... That
#: claim is WITHDRAWN") — and a test that banned the words outright would ban the
#: retraction along with the lie, leaving the honest surfaces unable to explain
#: themselves. So the rule is not "never say it"; it is **"never say it without taking it
#: back in the same breath"**.
WITHDRAWAL_MARKERS = ("withdrew", "withdrawn", "used to say", "retracted")

#: How far back to look for a withdrawal marker. One sentence's worth: the marker must be
#: attached to the claim, not merely somewhere else on the page.
WITHDRAWAL_WINDOW = 120

#: The opposite over-claim, which must not be shipped either. We do NOT know that
#: these joints wrap; we know we cannot see.
OVERCORRECTION = (
    "the other four wrap",
    "every joint wraps",
    "all six joints wrap",
)


def _flat(text: str) -> str:
    return re.sub(r"\s+", " ", text).lower()


def _asserts_the_claim(flat: str, claim: str) -> bool:
    """Does *flat* state *claim* as fact, rather than quoting it in order to withdraw it?"""
    for match in re.finditer(re.escape(claim), flat):
        window = flat[max(0, match.start() - WITHDRAWAL_WINDOW) : match.start()]
        if not any(marker in window for marker in WITHDRAWAL_MARKERS):
            return True
    return False


def _rezero_help() -> str:
    """Every scrap of help text ``arm101 arm rezero --help`` would print."""
    parser = _build_parser()
    # argparse hides the subparser map behind private attributes; walking it is the
    # only way to ask a parser what it would TELL somebody.
    (arm_action,) = [
        a for a in parser._actions if getattr(a, "choices", None) and "arm" in a.choices
    ]
    rezero = arm_action.choices["arm"]._actions
    (verb_action,) = [a for a in rezero if getattr(a, "choices", None) and "rezero" in a.choices]
    return verb_action.choices["rezero"].format_help()


def _all_surfaces() -> "dict[str, str]":
    """Every place this repo says anything to a human about re-zeroing.

    None of these may state the retracted claim — including the pages that only mention
    re-zero in passing.
    """
    return {
        **_surfaces_that_explain_which_joints(),
        "explain arm (noun index)": catalog.ENTRIES[("arm",)],
        "explain arm limits": catalog.ENTRIES[("arm", "limits")],
    }


def _surfaces_that_explain_which_joints() -> "dict[str, str]":
    """The surfaces whose JOB is to tell an operator which joints can be re-zeroed.

    These must not merely avoid the lie — they must carry the retraction, because they
    are where somebody goes to find out. (The `arm` noun index is a verb list; making it
    recite the whole retraction would be noise, so it is held only to the weaker bar.)
    """
    return {
        "learn (text)": learn._TEXT,
        "learn (json)": str(learn._as_json_payload()),
        "explain arm rezero": catalog.ENTRIES[("arm", "rezero")],
        "arm rezero --help": _rezero_help(),
    }


# ---------------------------------------------------------------------------
# The retraction
# ---------------------------------------------------------------------------


def test_no_operator_facing_surface_still_ASSERTS_the_retracted_claim() -> None:
    for where, text in _all_surfaces().items():
        flat = _flat(text)
        for claim in RETRACTED:
            assert not _asserts_the_claim(flat, _flat(claim)), (
                f"{where} still tells the operator {claim!r} as fact. Hardware (issue #43) "
                "contradicted it: three joints reached their commandable bound with NO "
                "contact, 2, 3 and 11 raw ticks from the seam. The honest line is "
                "'UNKNOWN, not unnecessary'. (Quoting the claim in order to WITHDRAW it is "
                "fine — that is what a retraction is.)"
            )


def test_the_retraction_is_not_replaced_by_the_OPPOSITE_over_claim() -> None:
    """We do not know that those joints wrap either. Do not say that we do."""
    for where, text in _all_surfaces().items():
        flat = _flat(text)
        for claim in OVERCORRECTION:
            assert claim not in flat, f"{where} over-corrected into {claim!r}"


def test_every_surface_that_explains_WHICH_JOINTS_says_UNKNOWN_NOT_UNNECESSARY() -> None:
    for where, text in _surfaces_that_explain_which_joints().items():
        assert "unknown, not unnecessary" in _flat(text), (
            f"{where} does not tell the operator the one thing that replaced the "
            "retracted claim."
        )


def test_wrist_rolls_refusal_survives_but_on_a_MEASURED_reason() -> None:
    """Issue #43 touched this one too — it just did not change the answer.

    The refusal stands (209 ticks is under the 300 a seam needs) but every surface must now
    give the operator the NUMBER, not the withdrawn impossibility. The distinction is not
    pedantry: "no arc exists" forecloses the question forever, while "the arc is 209 ticks"
    is a measurement, and measurements can be re-taken.
    """
    for where, text in _surfaces_that_explain_which_joints().items():
        flat = _flat(text)
        assert "wrist_roll" in flat, where
        assert "209" in flat, where  # the measured arc reaches the operator
    assert "209" in arm_spec.REZERO_UNKNOWN_HEADLINE
    assert "wrist_roll" in arm_spec._REZERO_REFUSED


def test_the_withdrawal_detector_is_not_vacuous() -> None:
    """The test above would be worthless if it called everything a withdrawal.

    So: the claim asserted flat is CAUGHT, and the claim quoted inside a withdrawal is
    NOT — which is exactly the distinction the whole module turns on.
    """
    claim = _flat(RETRACTED[1])
    assert _asserts_the_claim(f"the other four are fine: {claim}.", claim)
    assert not _asserts_the_claim(f"issue #43 withdrew the claim that {claim}.", claim)


# ---------------------------------------------------------------------------
# ...and it is RENDERED, so it cannot drift again
# ---------------------------------------------------------------------------


def test_the_headline_is_the_summarys_own_first_words__one_source_not_two() -> None:
    """Two constants, one claim. The short one is literally the long one's opening.

    A separate short paraphrase is a second thing to drift, which is the failure this
    whole test module exists to prevent.
    """
    assert arm_spec.REZERO_ARC_UNKNOWN_SUMMARY.startswith(arm_spec.REZERO_UNKNOWN_HEADLINE)


def test_the_rendered_wording_agrees_with_the_table_it_came_from() -> None:
    """The summary and the refusal message ``rezero_refusal`` actually returns are one claim."""
    refusal = _flat(arm_spec.rezero_refusal("wrist_flex") or "")
    assert "unknown, not unnecessary" in refusal
    assert "unknown, not unnecessary" in _flat(arm_spec.REZERO_ARC_UNKNOWN_SUMMARY)
    for claim in RETRACTED:
        # The refusal QUOTES the retracted claim in order to withdraw it — so the check
        # here is that it withdraws it, not that it never mentions it.
        if _flat(claim) in refusal:
            assert "withdrawn" in refusal


def test_the_surfaces_RENDER_the_wording_rather_than_restating_it() -> None:
    """Change ``arm_spec``'s wording and every operator-facing surface moves with it.

    Asserted by identity, not by similarity: the exact string from the table appears,
    character for character, in each surface that talks about the unmeasured joints.
    """
    summary = arm_spec.REZERO_ARC_UNKNOWN_SUMMARY
    assert summary in learn._TEXT
    assert summary in catalog.ENTRIES[("arm", "rezero")]
    assert arm_spec.REZERO_UNKNOWN_HEADLINE in _rezero_help().replace("\n", " ").replace(
        "  ", " "
    ) or arm_spec.REZERO_UNKNOWN_HEADLINE in " ".join(_rezero_help().split())
