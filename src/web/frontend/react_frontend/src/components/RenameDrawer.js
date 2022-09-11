import { useState } from 'react'
import PropTypes from 'prop-types'
import Drawer from './Drawer'
import Select from 'react-select'


const RenameDrawer = ({ renameDrawer, drawers }) => {
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
                const newDrawer=drawers.find(d=>d.id===selected_drawer_id)
                console.log(newDrawer)
                newDrawer.content=new_name
                renameDrawer(newDrawer)
                setselectedDrawerID(0)
                setNewName("")
                sessionStorage.setItem('tap-value', 1);
                window.parent.location.reload(false)
        }
        
        const options = [ ] 
              drawers.forEach((drawer)=>{
                
                options.push({value: drawer.id, label: drawer.drawer_controller_id + " (" + drawer.content + " )" }) 
            })
              
              
        return (
                <form className='rename-form' onSubmit={onSubmit}>
                        <div className='form-control'>
                        <label for="drawers">Schublade</label>
                        <Select options={options} onChange= {(event) => { setselectedDrawerID(event.value)}}/>
                        
                        </div>
                       {/*<div className='form-control'>
                                <label>Alter Name</label>
                                { selected_drawer_id>0? 
                                        (<label>{drawers.find(d=>d.id===selected_drawer_id).content} </label>):("")
                                }                                
                                
                        </div>*/}

                        <div className='form-control'>
                                <label> Füllung</label>
                                <input type='newTitle' placeholder= {selected_drawer_id>0? ( drawers.find(d=>d.id===selected_drawer_id).content ):("")}
                                        value={new_name} onChange={(e) => setNewName(e.target.value)} />
                        </div>

                        <input type='submit' value='Save' className='btn btn-block' />
                </form>
        )
}
RenameDrawer.propTypes = {
        renameDrawer: PropTypes.func,
        drawers: PropTypes.arrayOf(Drawer),
}
export default RenameDrawer

