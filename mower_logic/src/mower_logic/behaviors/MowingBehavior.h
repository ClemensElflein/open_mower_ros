// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#ifndef SRC_MOWINGBEHAVIOR_H
#define SRC_MOWINGBEHAVIOR_H

#include "Behavior.h"
#include "UndockingBehavior.h"

#include "slic3r_coverage_planner/PlanPath.h"
#include "slic3r_coverage_planner/Path.h"
#include "ftc_local_planner/PlannerGetProgress.h"

class MowingBehavior : public Behavior {

private:
    bool skip_area;
    bool create_mowing_plan(int area_index);

    bool execute_mowing_plan();

    // Progress
    bool mowerEnabled = false;
    std::vector<slic3r_coverage_planner::Path> currentMowingPaths;


public:
    static MowingBehavior INSTANCE;

    std::string state_name() override;

    Behavior *execute() override;

    void enter() override;

    void exit() override;

    void reset() override;

    bool needs_gps() override;

    bool mower_enabled() override;

    void command_home() override;

    void command_start() override;

    void command_s1() override;

    void command_s2() override;

    bool redirect_joystick() override;
};


#endif //SRC_MOWINGBEHAVIOR_H
