'''
Author: Qin Yang
05/08/2021
'''

import numpy as np
from fractions import Fraction
import time
import nashlh
import math
import scipy.stats as st


def first_level_mne(e, a, strategyProbablity):
    for s in range(len(strategyProbablity)):
        if s == 0:
            e['circle'] = strategyProbablity[s]
        elif s == 1:
            e['semicircle'] = strategyProbablity[s]
        elif s == 2:
            a['infiltrate'] = strategyProbablity[s]
        elif s == 3:
            a['escape'] = strategyProbablity[s]

    print(e)

    return (e, a)


def second_level_mne(e, a, strategyProbablity):
    for s in range(len(strategyProbablity)):
        if s == 0:
            e['pp'] = strategyProbablity[s]
        elif s ==1:
            e['cb'] = strategyProbablity[s]
        elif s == 2:
            a['change_direction'] = strategyProbablity[s]
        elif s == 3:
            a['change_speed'] = strategyProbablity[s]

    return (e, a)

# def first_level_mne(e, a, strategyProbablity):
#     for s in range(len(strategyProbablity)):
#         if s == 0:
#             e['attacking'] = strategyProbablity[s]
#         elif s == 1:
#             e['defending'] = strategyProbablity[s]
#         elif s == 2:
#             a['attacking'] = strategyProbablity[s]
#         elif s == 3:
#             a['defending'] = strategyProbablity[s]

#     print(e)

#     return (e, a)


# def second_level_mne(e, a, strategyProbablity):
#     for s in range(len(strategyProbablity)):
#         if s == 0:
#             e['pp'] = strategyProbablity[s]
#         elif s ==1:
#             e['cb'] = strategyProbablity[s]
#         elif s == 2:
#             a['change_direction'] = strategyProbablity[s]
#         elif s == 3:
#             a['change_speed'] = strategyProbablity[s]

#     return (e, a)

def GUT_CPT(e1_first_level_strategy, a1_first_level_strategy, second_level_mne, agent2_strategy):
    result = 0

    for item in second_level_mne.items():
        if item[0] == agent2_strategy:
            result = Fraction(e1_first_level_strategy) * Fraction(a1_first_level_strategy) * Fraction(item[1])
        
    return result

def final_explorer_strategy_combination(explorer_gut_cpt):
    tmp = sorted(explorer_gut_cpt.items(), key = lambda kv:(kv[1], kv[0]))[-1]

    if tmp[0] == 'e11a11e21t' or tmp[0] == 'e11a12e22t':
        return('semicircle', 'pp')
    elif tmp[0] == 'e11a11e21l' or tmp[0] == 'e11a12e22l':
        return('semicircle', 'cb')
    elif tmp[0] == 'e12a11e23t' or tmp[0] == 'e12a12e24t':
        return('circle', 'pp')
    elif tmp[0] == 'e12a11e23l' or tmp[0] == 'e12a12e24l':
        return('circle', 'cb')

def final_alien_strategy_combination(alien_gut_cpt):
    tmp = sorted(alien_gut_cpt.items(), key = lambda kv:(kv[1], kv[0]))[-1]

    if tmp[0] == 'e11a11a21t' or tmp[0] == 'e11a12a22t':
        return('infiltrate', 'change_direction')
    elif tmp[0] == 'e11a11a21l' or tmp[0] == 'e11a12a22l':
        return('infiltrate', 'change_speed')
    elif tmp[0] == 'e12a11a23t' or tmp[0] == 'e12a12a24t':
        return('escape', 'change_direction')
    elif tmp[0] == 'e12a11a23l' or tmp[0] == 'e12a12a24l':
        return('escape', 'change_speed')


def explorerAbility(explorersEnergyLevel, explorersHPLevel):
    a = 0.0111
    d = 0.0222

    explorerAttackingAbility = a * explorersEnergyLevel
    explorerDefendingAbility = d * explorersHPLevel

    return(explorerAttackingAbility, explorerDefendingAbility)

def alienAbility(aliensEnergyLevel, aliensHPLevel):
    a = 0.0107 * 2
    d = 0.0143 * 2

    alienAttackingAbility = a * aliensEnergyLevel
    alienDefendingAbility = d * aliensHPLevel

    return(alienAttackingAbility, alienDefendingAbility)


def WinningUtility(numExplorer, numActiveAlien, agent_energy_level, agent_hp_level, situation):
    explorers_energy_level = []
    explorers_HP_level = []
    aliens_energy_level = []
    aliens_HP_level = []
    attackingSituationCoefficient1 = 1.3
    attackingSituationCoefficient2 = 0.7
    defendingSituationCoefficient1 = 0.6
    defendingSituationCoefficient2 = 1.4
    # print(numActiveAlien)

    for i in range(numExplorer):
        explorers_energy_level.append(agent_energy_level[i])
        explorers_HP_level.append(agent_hp_level[i])

    aliens_energy_level = list(set(agent_energy_level).difference(set(explorers_energy_level)))
    aliens_HP_level = list(set(agent_hp_level).difference(set(explorers_HP_level)))

    explorerAttackingAbility, explorerDefendingAbility = explorerAbility(sum(explorers_energy_level), sum(explorers_HP_level))
    alienAttackingAbility, alienDefendingAbility = alienAbility(sum(aliens_energy_level), sum(aliens_HP_level))

    if situation == "10":
        winningUtility = math.pow((attackingSituationCoefficient1 * explorerAttackingAbility + attackingSituationCoefficient2 * explorerDefendingAbility) 
                                / (defendingSituationCoefficient1 * alienAttackingAbility + defendingSituationCoefficient2 * alienDefendingAbility), numExplorer / numActiveAlien)
    elif situation == "11":
        winningUtility = math.pow((attackingSituationCoefficient1 * explorerAttackingAbility + attackingSituationCoefficient2 * explorerDefendingAbility) 
                                / (attackingSituationCoefficient1 * alienAttackingAbility + attackingSituationCoefficient2 * alienDefendingAbility), numExplorer / numActiveAlien)
    elif situation == "01":
        winningUtility = math.pow((defendingSituationCoefficient1 * explorerAttackingAbility + defendingSituationCoefficient2 * explorerDefendingAbility) 
                                / (attackingSituationCoefficient1 * alienAttackingAbility + attackingSituationCoefficient2 * alienDefendingAbility), numExplorer / numActiveAlien)
    elif situation == "00":
        winningUtility = math.pow((defendingSituationCoefficient1 * explorerAttackingAbility + defendingSituationCoefficient2 * explorerDefendingAbility) 
                                / (defendingSituationCoefficient1 * alienAttackingAbility + defendingSituationCoefficient2 * alienDefendingAbility), numExplorer / numActiveAlien)

    return winningUtility

def HPUtility(numAttackingAgent, numAttackingAdversary, agent_hp_level, explorerStrategy, alienStrategy):
    explorers_HP_level = []
    aliens_HP_level = []
    agentRadius = 1
    agentArea = math.pow(agentRadius, 2) * math.pi
    explorerArea = 0
    alienArea = 0
    tmp1 = 0
    tmp2 = 0

    if explorerStrategy == 'pp' and alienStrategy == 'change_direction':
        explorerArea = agentArea * 1.5
        alienArea = agentArea * 1.2
    elif explorerStrategy == 'pp' and alienStrategy == 'change_speed':
        explorerArea = agentArea * 1.5
        alienArea = agentArea * 0.8
    elif explorerStrategy == 'cb' and alienStrategy == 'change_direction':
        explorerArea = agentArea * 0.8
        alienArea = agentArea * 1.2
    elif explorerStrategy == 'cb' and alienStrategy == 'change_speed':
        explorerArea = agentArea * 0.8
        alienArea = agentArea * 0.8

    for i in range(numAttackingAgent):
        explorers_HP_level.append(agent_hp_level[i])

    aliens_HP_level = list(set(agent_hp_level).difference(set(explorers_HP_level)))

    # for i in range(99):
    #     tmp1 = tmp1 + st.poisson(alienArea).pmf(i) * np.mean(explorers_HP_level) * i
    #     tmp2 = tmp2 + st.poisson(explorerArea).pmf(i) * np.mean(aliens_HP_level) * i

    # result = abs(numAttackingAgent * tmp1 - numAttackingAdversary * tmp2)

    result = abs(numAttackingAgent * alienArea * np.mean(explorers_HP_level) - numAttackingAdversary * explorerArea * np.mean(aliens_HP_level))

    return result

def GUT_DecisionMaking(numExplorer, numActiveAlien, agent_energy_level, agent_hp_level):
    # Building Attacking GUT Structure
    # ============================================================ Utility Matrix ============================================
    # Level 1 What -- attacking or defending (attacking) velocity selection
    m11 = round(WinningUtility(numExplorer, numActiveAlien, agent_energy_level, agent_hp_level, "10") * 100)
    m12 = round(WinningUtility(numExplorer, numActiveAlien, agent_energy_level, agent_hp_level, "11") * 100)
    m21 = round(WinningUtility(numExplorer, numActiveAlien, agent_energy_level, agent_hp_level, "01") * 100)
    m22 = round(WinningUtility(numExplorer, numActiveAlien, agent_energy_level, agent_hp_level, "00") * 100)

    n11 = round(WinningUtility(numActiveAlien, numExplorer, agent_energy_level, agent_hp_level, "10") * 100)
    n12 = round(WinningUtility(numActiveAlien, numExplorer, agent_energy_level, agent_hp_level, "11") * 100)
    n21 = round(WinningUtility(numActiveAlien, numExplorer, agent_energy_level, agent_hp_level, "01") * 100)
    n22 = round(WinningUtility(numActiveAlien, numExplorer, agent_energy_level, agent_hp_level, "00") * 100)  

    # situation1MatrixLevel1 = ['2 1\n4 3', '1 4\n4 1\n']
    situationMatrixLevel1 = [str(m11) + ' ' + str(m12) + '\n' + str(m21) + ' ' + str(m22), str(n11) + ' ' + str(n12) + '\n' + str(n21) + ' ' + str(n22) + '\n']

    x11 = round(HPUtility(numExplorer, numActiveAlien, agent_hp_level, 'pp', 'change_direction'))
    x12 = round(HPUtility(numExplorer, numActiveAlien, agent_hp_level, 'pp', 'change_speed'))
    x21 = round(HPUtility(numExplorer, numActiveAlien, agent_hp_level, 'cb', 'change_direction'))
    x22 = round(HPUtility(numExplorer, numActiveAlien, agent_hp_level, 'cb', 'change_speed'))

    y11 = round(HPUtility(numActiveAlien, numExplorer, agent_hp_level, 'pp', 'change_direction'))
    y12 = round(HPUtility(numActiveAlien, numExplorer, agent_hp_level, 'pp', 'change_speed'))
    y21 = round(HPUtility(numActiveAlien, numExplorer, agent_hp_level, 'cb', 'change_direction'))
    y22 = round(HPUtility(numActiveAlien, numExplorer, agent_hp_level, 'cb', 'change_speed'))

    # Level 2 How
    situationMatrixLevel2 = [str(x11) + ' ' + str(x12) + '\n' + str(x21) + ' ' + str(x22), str(y11) + ' ' + str(y12) + '\n' + str(y21) + ' ' + str(y22) + '\n']

    print(situationMatrixLevel1)
    print(situationMatrixLevel2)
    # =========================================================== GUT Strategy Selector =======================================
    strategyProbablity1 = nashlh.lhmne(situationMatrixLevel1)
    strategyProbablity2 = nashlh.lhmne(situationMatrixLevel2)

    e1 = {}
    e21 = {}
    e22 = {}
    e23 = {}
    e24 = {}

    a1 = {}
    a21 = {}
    a22 = {}
    a23 = {}
    a24 = {}

    explorer_gut_cpt = {}
    alien_gut_cpt = {}
    first_level_strategy = {}
    second_level_strategy = {}

    explorer_strategy = []
    alien_strategy = []

    # build mixed nash equilibrium (MNE) table
    # first level
    e1, a1 = first_level_mne(e1, a1, strategyProbablity1)

    # second level
    e21, a21 = second_level_mne(e21, a21, strategyProbablity2)
    e22, a22 = second_level_mne(e22, a22, strategyProbablity2)
    e23, a23 = second_level_mne(e23, a23, strategyProbablity2)
    e24, a24 = second_level_mne(e24, a24, strategyProbablity2)

    # calculate explorers' different strategies combination conditional probability
    explorer_gut_cpt['e11a11e21t'] = GUT_CPT(e1['semicircle'], a1['infiltrate'], e21, 'pp')
    explorer_gut_cpt['e11a11e21l'] = GUT_CPT(e1['semicircle'], a1['infiltrate'], e21, 'cb')
    explorer_gut_cpt['e11a12e22t'] = GUT_CPT(e1['semicircle'], a1['escape'], e22, 'pp')
    explorer_gut_cpt['e11a12e22l'] = GUT_CPT(e1['semicircle'], a1['escape'], e22, 'cb')

    explorer_gut_cpt['e12a11e23t'] = GUT_CPT(e1['circle'], a1['infiltrate'], e23, 'pp')
    explorer_gut_cpt['e12a11e23l'] = GUT_CPT(e1['circle'], a1['infiltrate'], e23, 'cb')
    explorer_gut_cpt['e12a12e24t'] = GUT_CPT(e1['circle'], a1['escape'], e24, 'pp')
    explorer_gut_cpt['e12a12e24l'] = GUT_CPT(e1['circle'], a1['escape'], e24, 'cb')

    # calculate aliens' different strategies combination conditional probability
    alien_gut_cpt['e11a11a21t'] = GUT_CPT(e1['semicircle'], a1['infiltrate'], a21, 'change_direction')
    alien_gut_cpt['e11a11a21t'] = GUT_CPT(e1['semicircle'], a1['infiltrate'], a21, 'change_speed')
    alien_gut_cpt['e11a12a22t'] = GUT_CPT(e1['semicircle'], a1['escape'], a21, 'change_direction')
    alien_gut_cpt['e11a12a22t'] = GUT_CPT(e1['semicircle'], a1['escape'], a21, 'change_speed')

    alien_gut_cpt['e12a11a23t'] = GUT_CPT(e1['circle'], a1['infiltrate'], a23, 'change_direction')
    alien_gut_cpt['e12a11a23t'] = GUT_CPT(e1['circle'], a1['infiltrate'], a23, 'change_speed')
    alien_gut_cpt['e12a12a24t'] = GUT_CPT(e1['circle'], a1['escape'], a24, 'change_direction')
    alien_gut_cpt['e12a12a24t'] = GUT_CPT(e1['circle'], a1['escape'], a24, 'change_speed')

    explorer_first_level_strategy, explorer_second_level_strategy = final_explorer_strategy_combination(explorer_gut_cpt)
    alien_first_level_strategy, alien_second_level_strategy = final_alien_strategy_combination(alien_gut_cpt)
    # final_strategy_combination(gut_cpt)

    explorer_strategy.append(explorer_first_level_strategy)
    explorer_strategy.append(explorer_second_level_strategy)

    alien_strategy.append(alien_first_level_strategy)
    alien_strategy.append(alien_second_level_strategy)

    print("pursuers' strategies are " + explorer_first_level_strategy + " and " + explorer_second_level_strategy)
    print("evaders' strategies are " + alien_first_level_strategy + " and " + alien_second_level_strategy)

    return(explorer_strategy, alien_strategy)