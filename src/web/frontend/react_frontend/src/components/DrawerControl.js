import * as React from 'react';
import PropTypes from 'prop-types'

import Drawers from './Drawers';
import RobastRobotFront from './RobastRobotFront'
import ButtonPopUp from './ButtonPopup';
import RenameDrawer from './RenameDrawer';
import RefillDrawers from './RefillDrawers'




const popUpModalRename= (renameDrawer, drawers)=> 
{
  return ( 
  <div>
    <h1>Edit Drawer</h1>
    {<RenameDrawer renameDrawer={renameDrawer} drawers={drawers}/>}
  </div>
    ); }

const popUpModalRefill= ( toggleEmpty, drawers )=>
{
  return(
    <div>
        <h1>Edit Drawer</h1>
        {<RefillDrawers drawers= {drawers} toggleEmpty={toggleEmpty} />}
    </div>
  );
}

const DrawerControl= ({drawers, user, openDrawer, renameDrawer, toggleEmpty})=> {
 
  return (
    <div>
      <RobastRobotFront/>
      <Drawers drawers={drawers} openDrawer={openDrawer} toggleEmpty= {toggleEmpty} user= {user}/>
      
       <ButtonPopUp name="rename_drawer" caption="Umbennen" popUp={popUpModalRename(renameDrawer, drawers)} />
      <ButtonPopUp name="refill_drawer" caption="Refill" popUp={popUpModalRefill(toggleEmpty, drawers)}/>

      
    </div>
    
  )
}
DrawerControl.propTypes = {
  drawer: PropTypes.array,
  user: PropTypes.object,
  openDrawer: PropTypes.func,
  renameDrawer: PropTypes.func,
  isEmpty: PropTypes.func,
  getDrawers: PropTypes.func,
}
export default DrawerControl