import PropTypes from 'prop-types'
import SimpleMap from './SimpleMap';
import MapPositions from './MapPositions';
import Button from '@mui/material/Button';
import { useState, useEffect } from 'react'
import ButtonPopUp from './ButtonPopup';
import AddMapPosition from './AddMapPosition';


const popupModal= (addMapPosition)=> {return ( 
<div>
  <h1>Add aposition</h1>
  {<AddMapPosition onAdd={addMapPosition}/>}
</div>
  ); }


export default function RobotControl({mapPositions, sendGoal,  StartRobot, StopRobot, HomeRobot, addMapPosition}) {
  
  return (
            <div class = "RobotControl">
              <div class ="ControlPanel">
                <Button id="Pause" class="NiceButton" variant="contained" onClick={StopRobot} >Pause</Button>
                <Button id="Resume" class="NiceButton" variant="contained" onClick={StartRobot} >Resume</Button>
                <Button id="Home" class="NiceButton" variant="contained" onClick={HomeRobot} >Home</Button>
              </div>

              <SimpleMap/>
              {mapPositions.length > 0 ? (
                <MapPositions mapPositions={mapPositions} sendGoal={sendGoal}/>
              ):('')
              }
            <br></br>  
            <ButtonPopUp name="AddPosition" caption="Neuer Punkt" popUp= {popupModal(addMapPosition)}/>
             

            </div>
          );
}
