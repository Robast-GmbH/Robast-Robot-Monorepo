import { useState } from 'react'
import PropTypes from 'prop-types'
import Drawer from './Drawer'




const RenameDrawer = ({ onEdit, drawers }) => {
        const [selected_drawer_id, setselectedDrawerID] = useState(0)
        const [new_name, setNewName] = useState('')
        const onSubmit = (e) => {
                e.preventDefault()
                if (!selected_drawer_id) {
                        alert('Wähle eine Schublade')
                        return
                }
                if (!new_name) {
                        alert('Wähle eien neuen Namen') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
               
                onEdit({ selected_drawer_id, new_name})
                setselectedDrawerID(0)
        }
        drawers= drawers.drawers;
        return (
                <form className='rename-form' onSubmit={onSubmit}>
                        <div className='form-control'>
                        <label for="drawers">Schublade</label>
                        <select onChange={(e) => setselectedDrawerID(e.target.value)} >
                                {drawers.map((drawer) => {
                                        {console.log(drawer)}
                                        <option value={drawer.id}>{drawer.name+" (Nr. "+drawer.id+") "}</option>
                                })}
                        </select>
                        </div>
                        <div className='form-control'>
                                <label>Alter Name</label>
                                <label>{drawers[selected_drawer_id].name}</label>
                                
                        </div>

                        <div className='form-control'>
                                <label>Neuer Name</label>
                                <input type='newTitle' placeholder='Bier'
                                        value={new_name} onChange={(e) => setNewName(e.target.value)} />
                        </div>

                        <input type='submit' value='Save' className='btn btn-block' />
                </form>
        )
}
RenameDrawer.propTypes = {
        onEdit: PropTypes.func,
        drawers: PropTypes.arrayOf(Drawer),
}
export default RenameDrawer

