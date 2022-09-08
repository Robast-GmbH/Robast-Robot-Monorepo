import * as React from 'react';
import Drawers from './Drawers';
import RobastRobotFront from './RobastRobotFront'
import RefillButton from './RefillButton'
import ButtonPopUp from './ButtonPopup';
import RenameDrawer from './RenameDrawer';

const popupModal= (renameDrawer,drawers)=> {return ( 
  <div>
    <h1>Edit Drawer</h1>
    {<RenameDrawer onAdd={renameDrawer} drawers={drawers}/>}
  </div>
    ); }

export default function DrawerControl({drawers, openDrawer,renameDrawer}) {
  console.log({drawers})
  return (
    <div>
      <RobastRobotFront/>
      <Drawers drawers={drawers} openDrawer={openDrawer}/>
      
      {/*<RefillButton drawers={drawers}/>*/}
      <ButtonPopUp name="rename_drawer" caption="Umbennen" popUp={popupModal({renameDrawer},{drawers})}/>
    </div>
  )
}
