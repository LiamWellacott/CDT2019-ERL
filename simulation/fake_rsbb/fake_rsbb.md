# fake_rsbb

This package pretends to be the Referee, Scoring and Benchmarking Box which is the device used in the competition to monitor the robot as it completes tasks.

## restricted_task_3.launch

launching this will cause the `/roah_rsbb/tablet/call` (I think the button granny annie presses to summon the robot) and `/roah_rsbb/tablet/position` (I think the topic used with the former to indicate where to be summoned too).

The robot software should be up before running this.