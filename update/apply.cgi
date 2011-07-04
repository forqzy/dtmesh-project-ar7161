#!/bin/sh
# jjPlus Corp. Copyright 2009

if [ -f "/tmp/server_ip" ] ; then
    RELOC=$(cat /tmp/server_ip)
else
    RELOC="192.168.1.2"
fi

if [ -f "/tmp/httpd_port" ] ; then
    PORT=":"$(cat /tmp/httpd_port)
else
    PORT=""
fi

cat <<EOF
Content-type: text/html


<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en" lang="en">
<head>
<style>
.mesWindow {
	border:#666 1px solid;
	background:#fff;
	}
.mesWindowTop {
	border-bottom:#eee 1px solid;
	margin-left:4px;
	padding:3px;
	font-weight:bold;
	text-align:left;
	font-size:12px;
	}
.mesWindowContent{
	margin:4px;
	font-size:12px;
	}
.mesWindow .close{
	height:15px;
	width:28px;
	border:none;
	cursor:pointer;
	text-decoration:underline;
	background:#fff
	}
</style>
<script>
//Message Window Function by Terry@JJplus.com
//Script start

var isIe=(document.all)?true:false;

function Position(ev)
{
  var ScreenX = document.documentElement.offsetWidth;
  var ScreenY = document.documentElement.offsetHeight;
  ScreenX = ScreenX / 2;
  ScreenY = ScreenY / 2;
  return {x:ScreenX, y:ScreenY};
}

function showMessageBox(wTitle,content,pos,wWidth)
{
  closeWindow();
  var bWidth=parseInt(document.documentElement.scrollWidth);
  var bHeight=parseInt(document.documentElement.scrollHeight);
  var back=document.createElement("div");
  back.id="back";
  var styleStr="background:#666;width:100%;height:100%;position:fixed;top:0;left:0;_position: absolute;_top:expression(documentElement.scrollTop+'px');_margin-top:0;";
  styleStr+=(isIe)?"filter:alpha(opacity=40);":"opacity:0.40;";
  back.style.cssText=styleStr+"z-index:1;";
  document.body.appendChild(back);
  var mesW=document.createElement("div");
  mesW.id="mesWindow";
  mesW.className="mesWindow";
  mesW.innerHTML="<div class='mesWindowTop'><table width='100%' height='100%'><tr><td align='center' class='bodyText'>"+wTitle+"</td><td style='width:1px;'></td></tr></table></div><div class='mesWindowContent' id='mesWindowContent'>"+content+"</div><div class='mesWindowBottom'></div>";
  styleStr="width:"+wWidth+"px;position:fixed;top:50%;left:50%;margin-top:-50px;margin-left:-"+(wWidth/2)+"px;_position: absolute;_top:expression(documentElement.scrollTop+"+bHeight/2+"-50+'px');_margin-top:0;";
  mesW.style.cssText=styleStr+"z-index:2;";
  document.body.appendChild(mesW);
}

function showBackground(obj,endInt)
{
  obj.filters.alpha.opacity+=1;
  if(obj.filters.alpha.opacity<endInt) setTimeout(function(){showBackground(obj,endInt)},8);
}

function closeWindow()
{
  if(document.getElementById('back')!=null)
    document.getElementById('back').parentNode.removeChild(document.getElementById('back'));
  if(document.getElementById('mesWindow')!=null)
    document.getElementById('mesWindow').parentNode.removeChild(document.getElementById('mesWindow'));
}

function timer(time,loc){
  var s = document.getElementById('countnum');
  var t = document.getElementById('timerbar');
  s.innerHTML = s.innerHTML - 1;
	var rate = Math.round(s.innerHTML * 100 / time)/100;
  if (s.innerHTML <= 0){
		t.rows[0].cells[1].width = 300;
		t.rows[0].deleteCell(2);
		t.rows[0].deleteCell(2);
		t.rows[0].deleteCell(2);
		t.rows[0].cells[2].width = 4;
    document.getElementById('counttext').innerHTML = 'Device ready.';
    window.location = loc; //need modified
  }else{
    var width1 = Math.round(300 * rate);
		if( width1 > 296 ) width1 = 295;
		var width0 = 296 - width1;
    t.rows[0].cells[1].width = width0;
    t.rows[0].cells[3].width = width1;
    setTimeout('timer('+time+',"'+loc+'")', 1000);
  }
}

function MessageBox(ev,msg,sec)
{
  var objPos = Position(ev);
	messContent="<div align='center' id='counttext'> Please wait for <span id='countnum'>"+sec+"</span> seconds. </div> <table align='center' width='308' height='30' id='timerbar' border='0' cellspacing='0' cellpadding='0'><tr height='30'><td background='/img/timebar_l.jpg' width='4'>&nbsp;</td><td background='/img/timebar_c.jpg' width='0'>&nbsp;</td><td background='/img/timebar_r.jpg' width='4'>&nbsp;</td><td background='/img/timebar_cw.jpg' width='296'>&nbsp;</td><td background='/img/timebar_rw.jpg' width='4'>&nbsp;</td><td background='/img/timebar_re.jpg' width='0'></td></tr>";
  showMessageBox(msg,messContent,objPos,350);
}

//Message Window Script End.
</script>
</head><body><script>
if(typeof parent.UpgradeDone == 'function') parent.UpgradeDone();
else{
	MessageBox(null,'Firmware Updating done.<br>System Rebooting...',65);
	setTimeout('timer(65, "http://$RELOC$PORT")', 1000);
}
</script></body></html>
EOF

#time to reboot, see you on the other side...
echo "Rebooting now" > /dev/console
/sbin/reboot -d 2 &
