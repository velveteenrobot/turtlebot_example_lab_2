#include "RRT.h"

#include "turtlebot_example.h"
#include "marker.h"

#include <tf/transform_datatypes.h>
#include <set>

#define RRT_COLOR 1
#define SELECTED_PATH_COLOR 0

/**
 * Propogates the dynamics for 1 step
 */
Pose propogateDynamics(Pose start, float speed, float turnRate) {
  Pose result = start;
  double roll, pitch, yaw;

  tf::Quaternion bt_q;
  quaternionMsgToTF(start.orientation, bt_q);
  tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw);

  result.position.x += CYCLE_TIME * speed * cos(yaw);
  result.position.y += CYCLE_TIME * speed * sin(yaw);
  yaw += CYCLE_TIME * turnRate;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(roll, pitch, yaw),
      result.orientation);
  return result;
}


Milestone::Milestone(
    Milestone* prev,
    Pose& endPose,
    float speed,
    float turnRate,
    int numCycles) :
    mPrevMilestone(prev),
    mEndPosition(endPose),
    mSpeed(speed),
    mTurnRate(turnRate),
    mNumCycles(numCycles) {}

static float randFloat(float min, float max) {
  return (max - min) * ((float)rand()) / ((float) INT_MAX) + min;;
}

/**
 * Use the weighted selction method to pick a milestone to expand
 * @param milestones sorted vector of milestones
 * @return one of the milestones, based on it's position in the list
 */
static Milestone* selectRandomMilestone(vector<Milestone*>& milestones) {
  const float decay = 0.25;
  for (int i = 0; i < milestones.size() - 1; ++i) {
    if (randFloat(0, 1) < decay) {
      return milestones[i];
    }
  }
  // base exit case
  return milestones.back();
}

static const int MAX_BRANCH_CREATION_ATTEMPS = 20;

static Milestone* makeRandomMilestone(Milestone* source, Map& map) {
  for (int i = 0; i < MAX_BRANCH_CREATION_ATTEMPS; ++i) {
    // TODO: normal distribution?
    // float speed = randFloat(-0.05, 0.3);
    float speed = randFloat(0.1, 0.3);
    float turnRate = randFloat(-1, 1);
    int numCycles = (rand() % 70) + 30;

    Pose endPose = source->getEndPose();
    bool failed = false;
    for (int cycle = 0; cycle < numCycles; ++cycle) {
      endPose = propogateDynamics(endPose, speed, turnRate);
      if (map.robotAreaOccupied(endPose)) {
        // this motion path wasn't possible
        failed = true;
        break;
      }
    }
    if (failed) {
      // try another set of values
      continue;
    }
    return new Milestone(source, endPose, speed, turnRate, numCycles);
  }
  return NULL;
}

static void insertMilestoneSorted(
    Milestone* newMilestone,
    vector<Milestone*>& milestones,
    Pose goal) {
  // most node expansions should be getting closer to the goal, so linear insert
  vector<Milestone*>::iterator iter = milestones.begin();
  float newDist = newMilestone->distTo(goal);
  while (iter != milestones.end() && (*iter)->distTo(goal) < newDist) {
    iter++;
  }
  milestones.insert(iter, newMilestone);
}

/**
 * Generate and draw a path to the goal
 * @param start The exact pose the path starts at
 * @param end the desired pose to end at
 * @param map reference to the map of the area being traveled
 * @return a list of milestones going form start to near end
 */
list<Milestone*> doRRT(Pose start, Pose end, Map& map) {
  // total exploitation of floats not being bery descrete :)
  vector<Milestone*> milestones;
  Milestone* finalMilestone = NULL;
  milestones.push_back(new Milestone(NULL, start, 0, 0, 0));
  for (;;) {
    // select a milestone
    Milestone* source = selectRandomMilestone(milestones);
    // select random motion inputs
    Milestone* newMilestone = makeRandomMilestone(source, map);
    if (newMilestone == NULL) {
      // that milestone didn't have any valid paths, try again
      continue;
    }

    if (newMilestone->distTo(end) < TARGET_ERROR) {
      finalMilestone = newMilestone;
      break;
    }
    // insert the new milestone
    insertMilestoneSorted(newMilestone, milestones, end);
    newMilestone->draw(RRT_COLOR);
  }

  list<Milestone*> result;
  set<Milestone*> dontDelete;
  for (Milestone *cur = finalMilestone;
       cur->getPrev() != NULL;
       cur = cur->getPrev()) {
    result.push_front(cur);
    dontDelete.insert(cur);
    cur->draw(SELECTED_PATH_COLOR);
  }

  //  free the unused milestones
  for (int i = 0; i < milestones.size(); ++i) {
    if (!dontDelete.count(milestones[i])) {
      delete milestones[i];
    }
  }

  return result;
}

float Milestone::distTo(Pose position) {
  float xDist = position.position.x - mEndPosition.position.x;
  float yDist = position.position.y - mEndPosition.position.y;
  return sqrt(xDist*xDist + yDist*yDist);
}

void Milestone::draw(int color) {
  Pose nextPose = mPrevMilestone->getEndPose();
  bool failed = false;
  vector<Point> points;
  points.push_back(nextPose.position);
  for (int cycle = 0; cycle < mNumCycles; ++cycle) {
    nextPose = propogateDynamics(nextPose, mSpeed, mTurnRate);
    points.push_back(nextPose.position);
  }
  drawLine(color, points);
}
