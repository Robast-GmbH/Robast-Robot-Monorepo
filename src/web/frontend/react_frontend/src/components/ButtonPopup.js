import PropTypes from 'prop-types'
import SimpleMap from './SimpleMap';
import MapPositions from './MapPositions';
import AddMapPosition from './AddMapPosition';
import Popup from 'reactjs-popup';

import Box from '@mui/material/Box';
import Button from '@mui/material/Button';
import Modal from '@mui/material/Modal';
import { useState, useEffect } from 'react'


const style = {
  position: 'absolute',
  top: '50%',
  left: '50%',
  padding:'200px',
  transform: 'translate(-50%, -50%)',
  width: 400,
  bgcolor: 'background.paper',
  border: '2px solid #FFF',
  boxShadow: 24,
  p: 4,

};


export default function ButtonPopUp({name, caption,  popUp }) {
  const [popupIsOpen, setPopupIsOpen] = useState(false)
  const closeModal = () => setPopupIsOpen(false);
  const openModal = () => setPopupIsOpen(true);
  
  return (
    <>
        <Button id={name} variant="contained" onClick={openModal} sx={{ mr: 3 }}>{caption}</Button>
        <Modal
          open={popupIsOpen}
          onClose={closeModal}
          aria-labelledby="modal-modal-title"
          aria-describedby="modal-modal-description"
        >
          <Box sx={style}>
           {popUp}
        </Box>
      </Modal>
    </>
          );
}ButtonPopUp.propTypes = {
  name: PropTypes.string,
  caption: PropTypes.string,
  popUp: PropTypes.object,
} 




//<>
         


/*<Button id={name+"_executor"}  variant="contained" onClick={openModal} >{caption}</Button>
            
              <Popup open={popupIsOpen} closeOnDocumentClick onClose={closeModal}>
                <>
                {popUp}
              
                </>
              </Popup>
            </>*/