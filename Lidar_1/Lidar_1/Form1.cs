using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Threading;

namespace Lidar_1
{
    public partial class Form1 : Form
    {
        double angle;
        double[,] SinCos = new double[760, 2];
        bool mode = false;
        StreamReader sr;
        string NameLidar = "";
        List<int> l = new List<int>();
        List<List<int>> nwPoints = new List<List<int>>();
        Color[] c = { Color.Black, Color.Red, Color.Purple, Color.Yellow, Color.Green,
                       Color.Blue, Color.Brown, Color.DarkOliveGreen, Color.Fuchsia, Color.Indigo,
                        Color.LightPink, Color.BlanchedAlmond, Color.DarkSlateBlue, Color.Khaki, Color.Maroon};

        public Form1()
        {
            InitializeComponent();
            angle = 270.0 / 760.0;
            for (int i = 0; i < SinCos.GetLength(0); i++)
            {
                double alpha = Math.PI * angle * i / 180;
                SinCos[i, 0] = Math.Sin(alpha);
                SinCos[i, 1] = Math.Cos(alpha);
            }

        }
        string[] part;
        int blockSize = 0;
        private void button1_Click(object sender, EventArgs e) // choose file in flooder
        {
            FolderBrowserDialog fbd = new FolderBrowserDialog();
            OpenFileDialog ofd = new OpenFileDialog();
            if (fbd.ShowDialog() == DialogResult.OK)
            {
                part = Directory.GetFiles(fbd.SelectedPath);
            }
            for (int i = 0; i < part.Length; i++)
            {
                comboBox1.Items.Add(part[i]);
            }
            dataGridView1.ColumnCount = 2;
            dataGridView1.Columns[0].Name = "Number";
            dataGridView1.Columns[1].Name = "Value";
            dataGridView1.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
        }
         private void comboBox1_SelectedIndexChanged(object sender, EventArgs e) //start when choose
         {
            NameLidar = comboBox1.Text; //what?
            sr = new StreamReader(NameLidar);
            mode = false;
            blockSize = Convert.ToInt32(textBox1.Text);
            button2.Text = "Pause";
            timer1.Start();

         }
         private void timer1_Tick_1(object sender, EventArgs e) //timer for reading
         {
            if (!mode)
            {
                List<int> data = new List<int>();
                dataGridView1.Rows.Clear();
                string line = sr.ReadLine();
                if (line != null)
                {
                    string[] parseLine = line.Split('>');
                    if (parseLine.Length != 1)
                    {
                        string[] StrData = parseLine[1].Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                        for (int i = 0; i < StrData.Length; i++)
                        {
                            if (StrData[i] != null)
                            {
                                data.Add(Convert.ToInt32(StrData[i]));
                                string[] row = { i.ToString(), StrData[i] };
                                dataGridView1.Rows.Add(row);
                            }
                        }
                        draw(data);
                    }
                }

            }
         }
        private void button2_Click(object sender, EventArgs e) //pause reading
        {
            if (mode)
            {
                mode = false;
                button2.Text = "Pause";
            }
            else
            {
                mode = true;
                button2.Text = "Play";
            }
        }

        int counter = 0;
        Bitmap deepMap;
        Graphics Deep;
        int[,] pointData = new int[760, 2];
        int[,] KlasterP = new int[760, 3];//point;klaster
        const int size = 20;
        int[,] mid = new int[size, 2];
        int[,] objects = new int[size, 4];
        void draw(List<int> inputD)
        {
            nwPoints.Clear();
            counter++;
            pictureBox1.Image = null;
            deepMap = new Bitmap(400, 400);
            int Center = 200;
            Deep = Graphics.FromImage(deepMap);
            for (int i = 0; i < inputD.Count; i++)
            {

                if (inputD[i] > 50)
                {
                    double point = (double)inputD[i] / 20;
                    int newX = (int)(Center - point * SinCos[i, 1]);
                    int newY = (int)(Center - point * SinCos[i, 0]);
                    pointData[i, 0] = newX;
                    pointData[i, 1] = newY;
                    nwPoints.Add(new List<int> { newX, newY });
                }
            }
            l.Clear();
            int iter = nwPoints.Count / blockSize; //what
            int ind;
            for (int instance = 0; instance < iter; instance++)
            {
                int pass = 0;
                ind = instance * blockSize;

                for (int step = 1; step < blockSize; step++)
                {
                    int index = instance * blockSize + step;
                    if (index < nwPoints.Count)
                    {
                        int sX = nwPoints[index - 1][0];
                        int sY = nwPoints[index - 1][1];
                        int cX = nwPoints[index][0];
                        int cY = nwPoints[index][1];
                        double deltaX = Math.Pow(cX - sX, 2);
                        double deltaY = Math.Pow(cY - sY, 2);
                        if (deltaX < 26 && deltaY < 26)
                        {
                            pass++;
                        }
                    }
                }
                if (pass == blockSize - 1)
                {
                    l.Add(ind);
                }
            }
            var OL = Data(l);
            for (int objl = 0; objl < OL.Count; objl++)
            {
                int start = OL[objl][0];
                int finish = start + OL[objl][1];
                int centerX = 0;
                int centerY = 0;
                if (OL[objl][1] > 41)
                {
                    for (int reg = start; reg < finish; reg++)
                    {
                        int x = nwPoints[reg][0];
                        int y = nwPoints[reg][1];
                        centerX += x;
                        centerY += y;
                        Deep.DrawRectangle(new Pen(new SolidBrush(Color.DarkOrchid)), x, y, 3, 3);
                    }
                }
                centerX = centerX / (finish - start);
                centerY = centerY / (finish - start);
                string listName = String.Format("StartLine: {0}, End:{1}, Center: ({2},{3})", start, finish, centerX, centerY);
                listBox1.Items.Add(listName);
                linecenters.Add(new List<int>() { centerX, centerY });
            }
            for (int cl = OL.Count - 1; cl > -1; cl--)
            {
                int index = OL[cl][0];
                int range = OL[cl][1];
                nwPoints.RemoveRange(index, range);
            }
            Klasters(nwPoints);
            for (int ptd = 0; ptd < KlasterP.GetLength(0); ptd++)
            {
                int index = KlasterP[ptd, 2];
                if (mid[index, 0] != -1)
                {
                    Deep.DrawRectangle(new Pen(new SolidBrush(c[index])), KlasterP[ptd, 0], KlasterP[ptd, 1], 2, 2);
                }
            }
            for (int cr = 0; cr < linecenters.Count; cr++)
            {
                Deep.FillEllipse(new SolidBrush(Color.Red), linecenters[cr][0], linecenters[cr][1], 6, 6);

            }

            for (int le = 0; le < lastLinecenters.Count; le++)
            {
                Deep.FillEllipse(new SolidBrush(Color.Blue), lastLinecenters[le][0], lastLinecenters[le][1], 6, 6);
            }

            lastLinecenters.Clear();
            for (int save = 0; save < linecenters.Count - 1; save++)
            {
                lastLinecenters.Add(linecenters[save]);
            }
            linecenters.Clear();
            pictureBox1.Image = deepMap;
            Deep.Dispose();
        }
        List<List<int>> linecenters = new List<List<int>>();
        List<List<int>> lastLinecenters = new List<List<int>>();
        List<List<int>> Data(List<int> elem)
        {
            listBox1.Items.Clear();
            List<List<int>> ListOut = new List<List<int>>();
            ListOut.Clear();
            int[] merge = new int[elem.Count];
            for (int r = 0; r < elem.Count - 1; r++)
            {
                int index = elem[r] + blockSize - 1;
                int ni = elem[r + 1];
                int sX = nwPoints[index][0];
                int sY = nwPoints[index][1];
                int cX = nwPoints[ni][0];
                int cY = nwPoints[ni][1];
                double deltaX = Math.Pow(cX - sX, 2);
                double deltaY = Math.Pow(cY - sY, 2);
                if (deltaX < 9 && deltaY < 9) 
                {
                    merge[r] = r + 1;
                }
            }

            for (int i = merge.Length; i > 0; i--)
            {
                if (i > 1)
                {
                    if (i == merge[i - 1])
                    {
                        if (merge[i] != 0)
                        {
                            merge[i - 1] = merge[i];
                        }
                        else
                        {
                            merge[i] = i;
                            merge[i - 1] = merge[i];
                        }
                    }
                }
                else
                {
                    if (merge[0] == i)
                    {
                        merge[0] = merge[i];
                    }
                }
            }
            for (int ol = 0; ol < merge.Length; ol++)
            {
                if (merge[ol] != 0)
                {
                    int kol = merge[ol] - ol;
                    int index = elem[ol];
                    ol = merge[ol];
                    int size = (kol + 1) * blockSize;
                    ListOut.Add(new List<int>() { index, (kol + 1) * blockSize });
                }
            }


            return ListOut;
        } //same obj like klass
        
        void Klasters(List<List<int>> values)
        {
            Random rnd = new Random();
            for (int i = 0; i < size; i++)
            {
                mid[i, 0] = rnd.Next(250, 350);
                mid[i, 1] = rnd.Next(250, 350);
            }
            for (int repeat = 0; repeat < 10; repeat++)
            {
                for (int j = 0; j < values.Count; j++)
                {
                    double distance = 0;
                    int index = 0;
                    for (int count = 0; count < size; count++)
                    {
                        if (count == 0)
                        {
                            double dist = Math.Sqrt(Math.Pow(((double)values[j][0] - (double)mid[count, 0]), 2)
                                + Math.Pow(((double)values[j][1] - (double)mid[count, 1]), 2));
                            distance = dist;
                            index = count;
                        }
                        else
                        {
                            double dist = Math.Sqrt(Math.Pow(((double)values[j][0] - (double)mid[count, 0]), 2)
                                + Math.Pow(((double)values[j][1] - (double)mid[count, 1]), 2));
                            if (distance > dist)
                            {
                                distance = dist;
                                index = count;
                            }
                        }

                    }
                    KlasterP[j, 0] = values[j][0];
                    KlasterP[j, 1] = values[j][1];
                    KlasterP[j, 2] = index;

                }
                for (int klas = 0; klas < size; klas++)
                {
                    int points = 0;
                    int x = 0;
                    int y = 0;
                    int max_x = 0, max_y = 0;
                    for (int v = 0; v < KlasterP.GetLength(0); v++)
                    {
                        if (KlasterP[v, 2] == klas)
                        {
                            x += KlasterP[v, 0];
                            y += KlasterP[v, 1];
                            points++;
                            if (max_x < KlasterP[v, 0])
                            {
                                max_x = KlasterP[v, 0];
                            }
                            if (max_y < KlasterP[v, 1])
                            {
                                max_y = KlasterP[v, 1];
                            }
                        }
                    }
                    int min_x = max_x, min_y = max_y;
                    for (int m = 0; m < KlasterP.GetLength(0); m++)
                    {
                        if (KlasterP[m, 2] == klas)
                        {

                            if (min_x > KlasterP[m, 0])
                            {
                                min_x = KlasterP[m, 0];
                            }
                            if (min_x > KlasterP[m, 1])
                            {
                                min_x = KlasterP[m, 1];
                            }
                        }
                    }
                    objects[klas, 0] = min_x;
                    objects[klas, 1] = min_y;
                    objects[klas, 2] = max_x;
                    objects[klas, 3] = max_y;

                    if (points != 0 && points > 10)
                    {
                        x = x / points;
                        y = y / points;
                        mid[klas, 0] = x;
                        mid[klas, 1] = y;
                    }
                    else
                    {
                        mid[klas, 0] = -1;
                    }
                }
            }
        }
        void Klasters(int[,] values)
        {
            Random rnd = new Random();
            for (int i = 0; i < size; i++)
            {
                mid[i, 0] = rnd.Next(200, 350);
                mid[i, 1] = rnd.Next(200, 350);
            }
            for (int repeat = 0; repeat < 10; repeat++)
            {
                for (int j = 0; j < values.GetLength(0); j++)
                {
                    double distance = 0;
                    int index = 0;
                    for (int count = 0; count < size; count++)
                    {
                        if (count == 0)
                        {
                            double dist = Math.Sqrt(Math.Pow(((double)values[j, 0] - (double)mid[count, 0]), 2)
                                + Math.Pow(((double)values[j, 1] - (double)mid[count, 1]), 2));
                            distance = dist;
                            index = count;
                        }
                        else
                        {
                            double dist = Math.Sqrt(Math.Pow(((double)values[j, 0] - (double)mid[count, 0]), 2)
                                + Math.Pow(((double)values[j, 1] - (double)mid[count, 1]), 2));
                            if (distance > dist)
                            {
                                distance = dist;
                                index = count;
                            }
                        }

                    }
                    KlasterP[j, 0] = values[j, 0];
                    KlasterP[j, 1] = values[j, 1];
                    KlasterP[j, 2] = index;

                }
                for (int klas = 0; klas < size; klas++)
                {
                    int points = 0;
                    int x = 0;
                    int y = 0;
                    for (int v = 0; v < KlasterP.GetLength(0); v++)
                    {
                        if (KlasterP[v, 2] == klas)
                        {
                            x += KlasterP[v, 0];
                            y += KlasterP[v, 1];
                            points++;
                        }
                    }
                    if (points != 0)
                    {
                        x = x / points;
                        y = y / points;
                        mid[klas, 0] = x;
                        mid[klas, 1] = y;
                    }
                }
            }
        }

    }
    

}
