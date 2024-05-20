## Consistency ##

### This thesis ###

В данной работе используется ...
В данной работе описывается ...

### References ###

Алгоритм, описанный в \cite{}
Похожая архитектура описана в \cite{}.

## Task ##

Выпускная работа бакалавра представляет собой выполненное под руководством опытного ученого законченное учебно-научное исследование, актуальное для современных физико-технических и математических проблем естествознания.

Работа должна содержать следующие основные разделы:
- обоснование выбора темы и ее актуальности
- физико-математическую постановку задачи
- обоснование выбора и изложение методов исследования и решения поставленной задачи
- анализ полученных результатов, список использованной литературы и выводы.

## Abstract ##

Данная работа посвящена решению задачи адаптивного рендеринга в приложениях реального времени.
В качестве цели работы была выбрана имплементация иерархического атласа с виртуальным текстурированием, одного из актуальных подходов к решению этой задачи.
В результате работы был выявлен и исправлен ряд недостатков и неточностей этой техники, и предложены улучшения оригинального метода.
Была написана на эффективная многопоточная реализация приложения конвертации моделей в формат иерархического атласа, а также приложение для рендеринга конвертированных моделей с использованием современного графического API.

## System ##

* overview
* optimizer objectives
  * tracking objective
  * linear balance
  * angular balance
* optimizer constraints
  * dynamics equation
  * support
  * friction cone
* forward dynamics
* implementation
  * pinocchio
  * cvxopt

## Minor tasks ##

Clean up style.tex

## Presentation ##

### Structure ###

1. ВВедение
2. Постановка задачи
3. Модель персонажа
4. Описание системы
5. Результаты

### Kinematic tree ###

Персонаж будет моделироваться как кинематическое дерево. Такой подход имеет ряд преимуществ:
* минимизируется количество чисел необходимых для описания положения всех тел
* задачи динамики имеет эффективные решения (за линейное время)

Оптимизация:
* позволяет генерировать движение одновременно основываясь на нескольких целях

## Implementation ##

we don't implement several things
* angular balance
* internal collisions

## Notation ##

### General ###

I identity
N normal
F friction

### Dynamics ###

A centroidal momentum matrix
H joint space inertia matrix
J jacobian
L angular momentum
M transformation
P linear momentum

### Optimization ###

minimize x^T Q x + 2 p^T q
subject to Bx = d

R regularization

### Coefficients ###

V tor
W weights

## LaTeX ##

Number equations

\begin{equation}
\end{equation}

Which kind of matrix should we use in diploma?
parentheses matrix or brackets matrix

O - Zero matrix
