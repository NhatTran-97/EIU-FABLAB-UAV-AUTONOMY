"""MissionExecutor: runs a compiled plan one step at a time.

Each plan entry belongs to a mission item (for progress reports). Task steps of one item share a group;
an entry may carry a fallback step. When a step with a fallback fails, the fallback runs instead and the
remaining skippable steps of its group are skipped (e.g. Search fails -> ManualHandover, then Ascend).
A failure without a fallback stops the plan.
"""

from dataclasses import dataclass, field

from mission_pipeline.steps import RUNNING, SUCCEEDED, Step

DONE = 'done'


@dataclass
class PlanEntry:
    item: int                       # 1-based mission item number
    step: Step
    group: int | None = None        # task group id (steps of one task at one waypoint)
    fallback: Step | None = field(default=None)
    skippable: bool = False         # skipped after a fallback of the same group has finished


class MissionExecutor:
    def __init__(self, plan, total_items):
        self.plan = list(plan)
        self.total = total_items
        self.index = 0
        self.step = None
        self.entry = None
        self.paused = False

    @property
    def current_item(self):
        return self.entry.item if self.entry else 0

    @property
    def current_step(self):
        return self.step

    def start(self, ctx):
        self.index = 0
        self._start_entry(ctx)

    def _start_entry(self, ctx):
        if self.index >= len(self.plan):
            self.entry, self.step = None, None
            return
        self.entry = self.plan[self.index]
        self.step = self.entry.step
        self.step.start(ctx)
        ctx.log(f'Mission item {self.entry.item}/{self.total}: {self.step.name}')

    def tick(self, ctx):
        """RUNNING, DONE, or Failed(reason) when the plan cannot continue."""
        if self.paused:
            return RUNNING
        if self.step is None:
            return DONE
        result = self.step.tick(ctx)
        if result == RUNNING:
            return RUNNING
        if result == SUCCEEDED:
            if self.step is not self.entry.step:          # a fallback finished: skip the rest of its group
                self._skip_group()
            else:
                self.index += 1
            self._start_entry(ctx)
            return DONE if self.step is None else RUNNING
        # failed
        self.step.abort(ctx)
        fallback = self.entry.fallback
        if fallback is not None and self.step is self.entry.step and not result.land:
            ctx.say(f'WP{self.entry.item} {self.step.name}: {result.reason}'[:50], warn=True)
            self.step = fallback
            self.step.start(ctx)
            ctx.log(f'Mission item {self.entry.item}: {result.reason} -> {self.step.name}', warn=True)
            return RUNNING
        return result

    def _skip_group(self):
        group = self.entry.group
        self.index += 1
        while group is not None and self.index < len(self.plan) and self.plan[self.index].group == group \
                and self.plan[self.index].skippable:
            self.index += 1

    def pause(self, ctx):
        """Stop the current step; resume() restarts it from the current position."""
        if self.step is not None and not self.paused:
            self.step.abort(ctx)
        self.paused = True

    def resume(self, ctx):
        if self.paused and self.step is not None:
            self.step.start(ctx)
        self.paused = False

    def abort(self, ctx):
        if self.step is not None:
            self.step.abort(ctx)
        self.step, self.entry = None, None
