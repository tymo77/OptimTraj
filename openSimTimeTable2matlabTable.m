function T = openSimTimeTable2matlabTable(TO)

Ncols = TO.getNumColumns();
Nrows = TO.getNumRows();
time_o = TO.getIndependentColumn();
Ntime = time_o.size();
assert(Ntime == Nrows);

colLabels = cell([1 Ncols]);
    

DATA = zeros([Nrows, Ncols + 1]);
for i = 1:Nrows
    DATA(i, 1) = time_o.get(i - 1);
    row = TO.getRowAtIndex(i - 1);
    for j = 1:Ncols
        DATA(i, 1 + j) = row.get(j - 1);
        colLabels{j} = TO.getColumnLabel(j - 1);
    end
end
T = array2table(DATA);
colLabels = string(colLabels);
T.Properties.VariableNames = ['time' colLabels];

end