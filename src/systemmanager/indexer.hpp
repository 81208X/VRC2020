// 
// Formally referred to as sorter system

#ifndef _INDEXER_HPP_INCLUDED
#define _INDEXER_HPP_INCLUDED

#include "api.h"
#include "profiles.hpp"
#include <algorithm>

#define DEFAULT_INDEXER_TIMEOUT 4000

class Indexer {
private:
    uint32_t taskStartTime;
    // Returns true if task has exceeded timeout
    bool checkTimeout(uint32_t timeout, uint32_t delay = 10);
    
public:
    bool gotUpperBall = false, gotLowerBall = false;

    /**
     * Feeds a ball into the upper position of the robot. This function is blocking
     * so be *absolutely* sure that you have some timeout if not the default value.
     * Vision is imperfect so *expect* this to fail occasionally.
     * 
     * \param timeout Timeout for attempting to get a ball in miliseconds.
     */
    void getUpperBall(uint32_t timeout = DEFAULT_INDEXER_TIMEOUT);
    /**
     * Feeds a ball into the lower position of the robot. This function is blocking
     * so be *absolutely* sure that you have some timeout if not the default value.
     * Vision is imperfect so *expect* this to fail occasionally.
     * 
     * \param timeout Timeout for attempting to get a ball in miliseconds.
     */
    void getLowerBall(uint32_t timeout = DEFAULT_INDEXER_TIMEOUT);
    /**
     * Moves a ball into the intake of the robot (for descore). This function is 
     * blocking so be *absolutely* sure that you have some timeout if not the 
     * default value. Vision is imperfect so *expect* this to fail occasionally.
     * 
     * \param timeout Timeout for attempting to get a ball in miliseconds.
     */
    void getIntakeBall(uint32_t timeout = DEFAULT_INDEXER_TIMEOUT);
    /**
     * Feeds a ball into the upper and lower positions of the robot, in that order. 
     * This function is blocking so be *absolutely* sure that you have some timeout
     * if not the default value. Vision is imperfect so *expect* this to fail 
     * occasionally.
     * 
     * \param timeout Timeout for getting both balls combined miliseconds.
     */
    void getBothBall(uint32_t timeout = DEFAULT_INDEXER_TIMEOUT);
    /**
     * Feeds a ball into the upper, lower, and intake positions of the robot, in
     * that order. This function is blocking so be *absolutely* sure that you have
     * some timeout if not the default value. Vision is imperfect so *expect* this
     * to fail occasionally.
     * 
     * \param timeout Timeout for getting both balls combined miliseconds.
     */
    void getAllBalls(uint32_t timeout = DEFAULT_INDEXER_TIMEOUT);
    /// Ejects ball in the intake of the robot. Used after descore.
    void discardLowerBall();
    /**
     * Scores balls detected in the upper and lower positions of the bot.
     *
     * \param speed Multiplier on the speed of the rollers. Must be <= 1;
     */
    void score(double speed = 1);

    /* ---------- INDEXER DEBUG FUNCTIONS ---------- */
    // These functions overide instructions from the rest of this wrapper
    // Do not reference in competition code!
    void debugIn();
    void debugOut();
    void debugStop();
};
#endif /* _INDEXER_HPP_INCLUDED */
