var currentTimeMs=(new Date()).getTime();
var websocket;

var output;

var dps1 = [];   //dataPoints. 
var dps2 = [];   //dataPoints. 
var dps3 = [];   //dataPoints. 
var x1=0;
var max1=-1000000;
var min1=1000000;
var chart1;

function handleEnablePID(cb)
{
	if(cb.checked)
		doSendCommand("enablePid");
	else
		doSendCommand("disablePid");
}

function keyPress(e)
{
	// look for window.event in case event isn't passed in
	e = e || window.event;
	if (e.keyCode == 13)
	{
		doSendCommand(e.srcElement.value);
		return false;
	}
	return true;
}		

function init()
{
	output = parent.document.getElementById("output");
	testWebSocket();
	
	chart1 = new CanvasJS.Chart("chartContainer1",{
	animationEnabled: true,
	zoomEnabled: true,
	title :{
		text: "Live Data"
	},
	axisX: {
		title: "timeline"
	},
	axisY: {						
		title: "Units"
	},
	data: [ 
		{
		type: "line",
		name: "pos1",
		lineThickness: 3,
		showInLegend: true,
		dataPoints : dps1,
		backgroundColor: "rgba(255,0,0,0.4)"
		},
		{
		type: "line",
		name: "pos2",
		lineThickness: 3,
		showInLegend: true,
		dataPoints : dps2,
		backgroundColor: "rgba(0,255,0,0.4)"
		},
		{
		type: "line",
		name: "pos_diff",
		lineThickness: 3,
		showInLegend: true,
		dataPoints : dps3,
		backgroundColor: "rgba(0,0,255,0.4)"
		}		
	  ]
	});
	
	
    chart1.render();	
	
}
function testWebSocket()
{
	if (location.host != "")
	{
		var wsUri = "ws://" + location.host + "/ws";
		//var wsUri = "ws://192.168.1.24/";
		websocket = new WebSocket(wsUri);
		websocket.onopen = function(evt) { onOpen(evt) };
		websocket.onclose = function(evt) { onClose(evt) };
		websocket.onmessage = function(evt) { onMessage(evt) };
		websocket.onerror = function(evt) { onError(evt) };		
	}
}
function onOpen(evt)
{
	writeToScreen("CONNECTED");
	writeToScreen('<span style="color: blue;">Time: ' + ((new Date()).getTime()-currentTimeMs) +'ms Received: ' + evt.data+'</span>');
}
function onClose(evt)
{
	writeToScreen("DISCONNECTED");
}

function onMessage(evt)
{
	if(evt.data.startsWith("{"))
		{
			var obj = JSON.parse(evt.data);
			//document.getElementById("encoder1_value").innerHTML = obj.encoder1_value;
			//document.getElementById("encoder2_value").innerHTML = obj.encoder2_value;
			//document.getElementById("esp32_heap").innerHTML = obj.esp32_heap;
			
			for(var propertyName in obj) {
			   // propertyName is what you want
			   // you can get the value like this: myObject[propertyName]
			   if(document.getElementById(propertyName) != null)
			   {
			   		if(document.getElementById(propertyName).nodeName == "INPUT")
			   			document.getElementById(propertyName).value = obj[propertyName];
			   		else	
						document.getElementById(propertyName).innerHTML = obj[propertyName];
			   }
			   
			   // chart data
			}
			drawChart(obj);
		}
	else
	{
		var allData = evt.data.split('\n');
		//writeToScreen('<span style="color: blue;">Time: ' + ((new Date()).getTime()-currentTimeMs) +'ms Received:</span>');
		for(i=0; i<allData.length; i++)
		{
			if(allData[i].startsWith("wifi ", 0))
			{
				writeToScreen("<span id='wifi" + i +"'>" + allData[i].split(' ')[1] + "</span><input id='pass" + i + "' width='300px'></input><button onclick='wifiConnect(" + i + ")'>Connect</button>");
			}
			else
			{
				writeToScreen('<span style="color: blue;">'+allData[i]+'</span>');
			}
		}
		var res = evt.data.split(' ');
		if (res.length==2 && res[0] == 'motor1_pos')
		{
			document.getElementById("motor1_pos").textContent = res[1];
		}
		if (res.length==2 && res[0] == 'motor2_pos')
		{		
			document.getElementById("motor2_pos").textContent = res[1];
		}
	}
}

function onError(evt)
{
	writeToScreen('<span style="color: red;">ERROR:</span> ' + evt.data);
}
function wifiConnect(i)
{
	textToSend = "wificonnect " + document.getElementById("wifi" + i).textContent + " " + document.getElementById("pass" + i).value;
	websocket.send(textToSend);
}
function doSend(element)
{
	//writeToScreen("SENT: " + message); 
	textToSend = element.value;
	websocket.send(textToSend);
}
function doSendParentElementId(element)
{

if(websocket!=null)
	websocket.send(element.id);
}
function doSendCommand(textToSend)
{

if(websocket!=null)
	websocket.send(textToSend);
}

function doSendCommand2(textToSend, element)
{

if(websocket!=null)
	websocket.send(textToSend + " " + element.id);
}

function writeToScreen(message)
{
	output.innerHTML = message + "<br>\n" + output.innerHTML;		
}
function doDisconnect()
{
	var disconnect = document.getElementById("disconnect");
	disconnect.disabled = true;
	websocket.close();
}

function jogClick(a)
{
	writeToScreen("SENT: " + a); 
	doSendCommand(a);		
}

function endsWith(str, suffix) {
	return str.indexOf(suffix, str.length - suffix.length) !== -1;
}

function drawChart(obj)
{
	if(obj.hasOwnProperty("encoder1_value"))
	{
		x1 = x1+1;
		//config.data.labels.push(newDate(config.data.labels.length));
		dps1.push({x: x1,y: parseFloat(obj.encoder1_value)});
		dps2.push({x: x1,y: parseFloat(obj.encoder2_value)});
		dps3.push({x: x1,y: parseFloat(obj.actual_diff)});

		if (dps1.length >  100 )
		{
			dps1.shift();					
		}
		if (dps2.length >  100 )
		{
			dps2.shift();					
		}
		
		if (dps3.length >  100 )
		{
			dps3.shift();					
		}
		
		var max_01 = maxValue(chart1.options.data[0].dataPoints);
		var max_02 = maxValue(chart1.options.data[1].dataPoints);
		chart1.options.axisY.maximum = Math.max(max_01, max_02);
		
		var min_01 = minValue(chart1.options.data[0].dataPoints);
		var min_02 = minValue(chart1.options.data[1].dataPoints);
		var min_03 = minValue(chart1.options.data[2].dataPoints);
		min_04 = Math.min(min_01, min_02);
		
		chart1.options.axisY.minimum =Math.min(min_04, min_03)-5;
		
		
		chart1.render();
		//chart2.render();				   
	}
}

function now() {
	return moment().toDate();
}

var maxValue = function(dataPoints) {
	var max1 = dataPoints[0].y;
	for (i = 0; i < dataPoints.length; i++) {
		if (dataPoints[i].y > max1) {
		  max1 = dataPoints[i].y;
		}
	}
	//return Math.round(maximum);
	return Math.ceil(max1/10)*10;
};


var minValue = function(dataPoints) {
	var min1 = dataPoints[0].y;
	for (i = 0; i < dataPoints.length; i++) {
		if (dataPoints[i].y < min1) {
		  min1 = dataPoints[i].y;
		}
	}
	//return Math.round(maximum);
	return Math.ceil(min1/10)*10;
};  

window.addEventListener("load", init, false);
