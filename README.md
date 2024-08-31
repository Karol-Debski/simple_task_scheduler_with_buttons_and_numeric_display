# simple_task_scheduler_with_buttons_and_numeric_display
<img src="https://drive.google.com/uc?export=view&id=1TI-1tsBFB9iUWzUE9fsJVxX1ay3GiRx0" alt="Przykładowy obrazek" width="500" />


Projekt implementacji schedulera bazującego na rejestrze MSP (Main Stack Pointer) i PSP (Process Stack Pointer) z mechanizmem zapamiętywania kontekstu tasków.

Scheduler mieści się w funkcji odpowiadającej za przerwanie dla 3 przycisków, które są powiazane z 3 taskami. Scheduler i taski mają swoje indywidualne stosy. W momencie przełączania się tasków, stan tasku poprzedniego (rejestry rdzenia procesora) jest zapisywany na swoim prywatnym stosie, a stan tasku uruchamianego jest odzyskiwany ze swojego prywatnego stosu. Wywłaszczanie występuje w momencie wystąpienia przerwania na którymś z 3 przycisków.  
