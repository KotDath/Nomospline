
Граничное представление, или сокращенно B-rep, можно рассматривать как расширение каркасной модели. Достоинство B-представления в том, что твердое тело ограничено своей поверхностью и имеет внутреннюю и внешнюю часть. Поверхность твердого тела состоит из набора хорошо организованных граней, каждая из которых представляет собой кусок некоторой поверхности (например, участок поверхности). Грани могут иметь общие вершины и ребра, которые являются сегментами кривой. Таким образом, B-rep является расширением каркасной модели за счет добавления к последней информации о лице.

В B-представлении есть два типа информации: топологическая и геометрическая. Топологическая информация обеспечивает отношения между вершинами, ребрами и гранями, аналогичные тем, которые используются в каркасной модели. Помимо связности топологическая информация также включает ориентацию ребер и граней. Геометрическая информация обычно представляет собой уравнения ребер и граней. Граничное представление, или сокращенно B-rep, можно рассматривать как расширение каркасной модели. Достоинство B-представления в том, что твердое тело ограничено своей поверхностью и имеет внутреннюю и внешнюю часть. Поверхность твердого тела состоит из набора хорошо организованных граней, каждая из которых представляет собой кусок некоторой поверхности (например, участок поверхности). Грани могут иметь общие вершины и ребра, которые являются сегментами кривой. Таким образом, B-rep является расширением каркасной модели за счет добавления к последней информации о лице.

В B-представлении есть два типа информации: топологическая и геометрическая. Топологическая информация обеспечивает отношения между вершинами, ребрами и гранями, аналогичные тем, которые используются в каркасной модели. Помимо связности топологическая информация также включает ориентацию ребер и граней. Геометрическая информация обычно представляет собой уравнения ребер и граней.

В нашей задаче грани задаются при помощи NURBS сплайнов


## Булевы операции

Шаги в реализации булевых операций на BREP:

1. Создаём Boundary Box для каждого сплайна. Если они попарно не пересекаются, значит точек пересечения нет и можно пропустить
2. 
3. Попарно перебираем все NURBS поверхности
4. Находим их пересечения
5. Все точки пересечения соотносим к соотв. поверхностям
6. 