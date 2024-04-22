// TODO getYt
yt_raw = 0;
for(Msg msg1:msgs)
{
//	yt_raw += msg1.delta_msgLength;
	if(msg1.sourceID==msg1.targetID&&msg1.sourceID!=-1)
	{
		yt_raw++;
		msg1.removeFlag=true;
	}
}

//Rtotal 添加部分
//if(averageLengthOfMsgs!=0)
	//yt_raw = yt_raw/averageLengthOfMsgs*5;
file_ytraw.print(yt_raw);
file_ytraw.print("  ");
yts_raw.add(yt_raw);
double c = (double)1/(2*M+1);
int end = yts_raw.size();
if(end>(2*M+1))
{
	double gi = 0;
	for(int i = 0;i<(2*M+1);i++)
	{
		gi += yts_raw.get(end-1-i);
	}
	gi = gi*c;
	yt = gi;//计算所得为当前时刻往前M个时刻的数据
	file_yt.print(yt);
	file_yt.print("  ");
	yts.add(yt);
	double NR = yts_raw.get(end-1-M) - yt;//计算所得为当前时刻往前M个时刻的数据
	noise.add(NR);
}  


// TODO   getRi
//每200s发生一次打击恢复，计算一次ri，选取感兴趣时间间隔为200s，其中低估100两端各50
//统计工作，后期的参数计算做准备
double Ps=0;
double Pn = 0;
double sum_yt = 0;
double sum_yd = 0;
double sum_ymin = 0;
double sum_yR = 0;
for(int i = 0;i<yts.size();i++)
{
	double temp = yts.get(i);
	sum_yt += temp;
	if(i<intervalOfRemove*0.2)
		sum_yd += temp;
	if(i>=intervalOfRemove*0.3&&i<intervalOfRemove*0.7)
		sum_ymin += temp;
	if(i>=intervalOfRemove*0.73&&i<intervalOfRemove*0.98)
		sum_yR += temp;
	Ps+=temp*temp;
	Pn+=noise.get(i)*noise.get(i);
}

//计算各参数
double SNRdB = 10 * log10( Ps/Pn);
double yd = sum_yd/intervalOfRemove/0.2;
double ymin = sum_ymin/intervalOfRemove/0.4;
double yR = sum_yR/intervalOfRemove/0.25;
//计算Performance factor
double d = sum_yt/yd/yts.size();
double delta = ymin/yd;
double lou = yR/yd;
double tao = (intervalOfRemove+4)/2/yts.size();
double J = 1/(1+exp(-0.25*(SNRdB-15)));

double	Ri = 0;
if(lou < delta)
	Ri = d*lou*(delta+J);
else
	Ri = d*lou*(delta+J+1-pow(tao,lou-J));
Ris.add(Ri);file_R.print(Ri);file_R.print(" ");
yts.clear();
noise.clear();

//十次打击后计算Rtotal
if(Ris.size() == 10)
{
	double alfa = 0.06;
	double sum_wi = 0;
	for(int i = 0;i<Ris.size();i++)
		sum_wi+=pow(1-alfa,9-i);
	for(int i = 0;i<Ris.size();i++)
	{
		double wi = pow(1-alfa,9-i);
		Rtotal+=Ris.get(i)*wi/sum_wi;
	}
	file_Rtotal.print(Rtotal);file_Rtotal.print(" ");
}


// Uav.msgGenerator
if(randomTrue(main.msgGenerateProb))
{

	Msg msg = main.add_msgs();
	MsgSource_target mst = getSourceTarget();
	msg.sourceID = mst.sourceUav;
	msg.targetID = mst.targetUav;
	msg.rootSourceID = msg.sourceID;
	main.msgCount++;
}
