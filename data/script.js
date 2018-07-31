var currentTimeMs=(new Date()).getTime();
var websocket;

var output;

var xmlhttp = new XMLHttpRequest();

function searchKeyPress2(e)
{
	// look for window.event in case event isn't passed in
	e = e || window.event;
	if (e.keyCode == 13)
	{
		document.getElementById('btn2').click();
		return false;
	}
	return true;
}

function handleEnableMotors(cb)
{
	if(cb.checked)
		doSendCommand("enable");
	else
		doSendCommand("disable");
}

function searchKeyPress1(e)
{
	// look for window.event in case event isn't passed in
	e = e || window.event;
	if (e.keyCode == 13)
	{
		document.getElementById('btn1').click();
		return false;
	}
	return true;
}		

function init()
{
	output = parent.document.getElementById("output");
	testWebSocket();

	xmlhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			myObj = JSON.parse(this.responseText);
			txt += "<select>"
			for (x in myObj) {
				txt += "<option>" + myObj[x].name;
			}
			txt += "</select>"
			document.getElementById("demo").innerHTML = txt;
		}
	}
	makePeriodicJsonReq();
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

window.addEventListener("load", init, false);

function makePeriodicJsonReq() {
	obj = { "table":"customers", "limit":20 };
	dbParam = JSON.stringify(obj);
	xmlhttp.open("POST", "/json", true);
	xmlhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
	xmlhttp.send("x=" + dbParam);
	setTimeout(makePeriodicJsonReq, 3000);
}
