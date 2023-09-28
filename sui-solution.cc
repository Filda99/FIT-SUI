#include "search-strategies.h"
#include <vector>
#include <stack>
#include "memusage.h"
#include <algorithm>
#include <unordered_set>
#include <iostream>

#include "search-strategies.h"
#include <iostream>
#include <deque>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include "memusage.h"

using namespace std;

struct State
{
	shared_ptr<SearchState> node;
	vector<shared_ptr<SearchAction>> actions;
};

vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state)
{
	if (init_state.isFinal()) return {};

	deque<State> open;
	unordered_set<shared_ptr<SearchState>> closed;

	// Initial state
	State initInfo = {make_shared<SearchState>(init_state), {}};
	open.push_back(move(initInfo));

	// If the final solution is found, we want to return the solution
	bool isSolutionFound = false;

	// Cycle through the tree
	while (!open.empty() && !isSolutionFound)
	{
		shared_ptr<SearchState> currentState = move(open.back().node);
		vector<shared_ptr<SearchAction>> currentStateActions = move(open.back().actions);
		open.pop_back();

		closed.insert(currentState);

		// Save all child-nodes to open
		for (const SearchAction &action : currentState->actions())
		{
			shared_ptr<SearchState> nextState = make_shared<SearchState>(move(action.execute(*currentState)));
			if (getCurrentRSS() > (mem_limit_ * 0.90))
			{
				return {};
			}
			// Insert only not visited nodes
			if (closed.find(nextState) == closed.end())
			{
				vector<shared_ptr<SearchAction>> actionPath = currentStateActions;
				actionPath.push_back(make_shared<SearchAction>(action));
				open.push_front({nextState, move(actionPath)});
				if (nextState->isFinal())
				{
					isSolutionFound = true;
					break;
				}
			}
		}
	}

	// Create path to final node
	vector<SearchAction> solution = {};
	if (!open.empty() && open.front().node->isFinal())
	{
		// Solution is saved in last node->actions (which handles path from init node to final one)
		for (shared_ptr<SearchAction> &action : open.front().actions)
		{
			solution.push_back(*action);
		}
		return solution;
	}
	else
	{
		return {};
	}
}

vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state)
{
	if (init_state.isFinal()) return {};

	deque<State> open;
	unordered_set<shared_ptr<SearchState>> closed;

	// Initial state
	State initInfo = {make_shared<SearchState>(init_state), {}};
	open.push_back(move(initInfo));

	// If the final solution is found, we want to return the solution
	bool isSolutionFound = false;

	// Cycle through the tree
	while (!open.empty() && !isSolutionFound)
	{
		if (open.back().actions.size() < depth_limit_)
		{
			shared_ptr<SearchState> currentState = move(open.back().node);
			vector<shared_ptr<SearchAction>> currentStateActions = move(open.back().actions);
			open.pop_back();

			closed.insert(currentState);

			// Save all child-nodes to open
			for (const SearchAction &action : currentState->actions())
			{
				shared_ptr<SearchState> nextState = make_shared<SearchState>(move(action.execute(*currentState)));

				if (getCurrentRSS() > (mem_limit_ * 0.90))
				{
					return {};
				}

				// Insert only not visited nodes
				if (closed.find(nextState) == closed.end())
				{
					vector<shared_ptr<SearchAction>> actionPath = currentStateActions;
					actionPath.push_back(make_shared<SearchAction>(action));
					open.push_back({nextState, move(actionPath)});
					if (nextState->isFinal())
					{
						isSolutionFound = true;
						break;
					}
				}
			}
		}
		else
		{
			open.pop_back();
		}
	}

	// Create path to final node
	vector<SearchAction> solution = {};
	if (!open.empty() && open.back().node->isFinal())
	{
		// Solution is saved in last node->actions (which handles path from init node to final one)
		for (shared_ptr<SearchAction> &action : open.back().actions)
		{
			solution.push_back(*action);
		}
		return solution;
	}
	else
	{
		return {};
	}
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const
{
	return 0;
}

vector<SearchAction> AStarSearch::solve(const SearchState &init_state)
{
	return {};
}
