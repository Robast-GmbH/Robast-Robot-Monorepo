import * as React from 'react';
import PropTypes from 'prop-types'

import Drawers from './Drawers';
import RobastRobotFront from './RobastRobotFront'
import ButtonPopUp from './ButtonPopup';
import RenameDrawer from './RenameDrawer';
import RefillDrawer from './RefillDrawer'




const popUpModalRename= ({renameDrawer, drawers})=> 
{
  return ( 
  <div>
    <h1>Edit Drawer</h1>
    {console.log(drawers)}
    {<RenameDrawer renameDrawer={renameDrawer} drawers={drawers}/>}
  </div>
    ); }

const popUpModalRefill= ( {toggleEmpty, drawers} )=>
{
  return(
    <div>
        <h1>Edit Drawer</h1>
        {<RefillDrawer drawers={drawers} toggleEmpty={toggleEmpty} />}
    </div>
  );
}

const DrawerControl= ({drawers, openDrawer, renameDrawer, toggleEmpty})=> {
  console.log({drawers})
  return (
    <div>
      <RobastRobotFront/>
      <Drawers drawers={drawers} openDrawer={openDrawer} toggleEmpty= {toggleEmpty}/>
      <ButtonPopUp name="rename_drawer" caption="Umbennen" popUp={popUpModalRename({renameDrawer}, {drawers})}/>

      <ButtonPopUp name="refill_drawer" caption="Refill" popUp={popUpModalRefill({toggleEmpty}, {drawers})}/>
    </div>
    
  )
}
DrawerControl.propTypes = {
  drawer: PropTypes.array,
  openDrawer: PropTypes.func,
  renameDrawer: PropTypes.func,
  isEmpty: PropTypes.func,
  getDrawers: PropTypes.func,
}
export default DrawerControl